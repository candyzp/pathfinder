#include <Geode/Geode.hpp>
#include "solver_dashboard.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

using namespace geode::prelude;

namespace {

struct DashboardStateV8 {
    PathfinderTelemetry telemetry;
    double trialsPerSecond = 0.0;
    uint64_t lastTrials = 0;
    std::chrono::steady_clock::time_point lastRateSample = std::chrono::steady_clock::now();
    bool hasTelemetry = false;
};

std::mutex s_dashboardMutex;
DashboardStateV8 s_dashboard;
std::atomic_bool s_tickerQueued = false;

char const* vehicleNameV8(int type) {
    switch (type) {
        case 0: return "Cube";
        case 1: return "Ship";
        case 2: return "Ball";
        case 3: return "UFO";
        case 4: return "Wave";
        case 5: return "Robot";
        case 6: return "Spider";
        case 7: return "Swing";
        default: return "Unknown";
    }
}

std::string compactTextV8(std::string value, size_t maxLength = 58) {
    for (char& c : value) {
        if (c == '\n' || c == '\r' || c == '\t')
            c = ' ';
    }
    if (value.empty())
        return "-";
    if (value.size() <= maxLength)
        return value;
    if (maxLength < 4)
        return value.substr(0, maxLength);
    return value.substr(0, maxLength - 3) + "...";
}

class PathfinderDashboardTickerV8 : public CCObject {
public:
    void tick(float) {
        DashboardStateV8 state;
        {
            std::lock_guard<std::mutex> guard(s_dashboardMutex);
            if (!s_dashboard.hasTelemetry)
                return;
            state = s_dashboard;
        }

        auto* director = CCDirector::sharedDirector();
        if (!director)
            return;
        auto* scene = director->getRunningScene();
        if (!scene)
            return;

        auto* original = typeinfo_cast<CCLabelBMFont*>(
            scene->getChildByIDRecursive("solver-debug")
        );
        if (!original)
            return;

        auto* parent = original->getParent();
        if (!parent)
            return;

        auto* dashboard = typeinfo_cast<CCLabelBMFont*>(
            parent->getChildByID("solver-dashboard-v8")
        );
        if (!dashboard) {
            dashboard = CCLabelBMFont::create("", "chatFont.fnt");
            if (!dashboard)
                return;
            dashboard->setID("solver-dashboard-v8");
            dashboard->setAnchorPoint({0.5f, 0.5f});
            dashboard->setScale(0.31f);
            dashboard->setPosition(original->getPosition() + ccp(0.f, -2.f));
            parent->addChild(dashboard, original->getZOrder() + 1);
        }

        // Keep the stock label as a lifecycle signal. PathfinderNode hides it
        // when the run ends, so the richer dashboard disappears automatically.
        dashboard->setVisible(original->isVisible());
        if (!original->isVisible())
            return;
        original->setOpacity(0);

        auto const& t = state.telemetry;
        int alive = std::max(0, t.producedCount);
        int rejected = std::max(0, t.duplicateCount + t.deadCount);
        std::string focus = t.stallRescue
            ? fmt::format("X {:.0f} ({} repeats)", t.focusX, t.deathClusterCount)
            : "none";
        std::string behavior = compactTextV8(
            !t.decision.empty() ? t.decision : t.recoveryReason
        );
        std::string why = compactTextV8(t.recoveryReason);

        auto text = fmt::format(
            "PATHFINDER BRAIN v8\n"
            "DOING  {}\n"
            "WHY    {}\n"
            "BEST   {:.2f}% | {} | X {:.0f} | clear {:.1f}\n"
            "POP    {} states: {} guided + {} explore | frontier {}\n"
            "SEARCH P{} | horizon {}f | archive {} | focus {}\n"
            "RESULT {} alive -> {} unique | {} rejected | {} dead\n"
            "STALL  {} layers | recoveries {} | {} physical threads\n"
            "SPEED  {:.0f} trials/s | {:L} total trials",
            behavior,
            why,
            t.progress,
            vehicleNameV8(t.vehicleType),
            t.currentX,
            t.bestClearance,
            t.workerCount,
            t.guidedCount,
            t.explorerCount,
            t.frontierCount,
            t.searchLevel,
            t.horizonFrames,
            t.archiveCount,
            focus,
            alive,
            t.uniqueCount,
            rejected,
            t.deadCount,
            t.stallLayers,
            t.recoveryCount,
            t.physicalThreadCount,
            state.trialsPerSecond,
            t.totalTrials
        );
        dashboard->setString(text.c_str());
    }
};

PathfinderDashboardTickerV8* s_ticker = nullptr;

void ensureTickerV8() {
    if (s_ticker || s_tickerQueued.exchange(true))
        return;

    Loader::get()->queueInMainThread([] {
        s_tickerQueued = false;
        if (s_ticker)
            return;
        auto* director = CCDirector::sharedDirector();
        if (!director || !director->getScheduler())
            return;

        s_ticker = new PathfinderDashboardTickerV8();
        s_ticker->retain();
        director->getScheduler()->scheduleSelector(
            schedule_selector(PathfinderDashboardTickerV8::tick),
            s_ticker,
            0.075f,
            false
        );
    });
}

} // namespace

void publishPathfinderTelemetryV8(PathfinderTelemetry const& telemetry) {
    auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> guard(s_dashboardMutex);
        auto elapsed = std::chrono::duration<double>(
            now - s_dashboard.lastRateSample
        ).count();
        if (elapsed >= 0.10) {
            uint64_t delta = telemetry.totalTrials >= s_dashboard.lastTrials
                ? telemetry.totalTrials - s_dashboard.lastTrials
                : 0;
            double instant = static_cast<double>(delta) / elapsed;
            if (s_dashboard.trialsPerSecond <= 0.0)
                s_dashboard.trialsPerSecond = instant;
            else
                s_dashboard.trialsPerSecond =
                    s_dashboard.trialsPerSecond * 0.72 + instant * 0.28;
            s_dashboard.lastTrials = telemetry.totalTrials;
            s_dashboard.lastRateSample = now;
        }
        s_dashboard.telemetry = telemetry;
        s_dashboard.hasTelemetry = true;
    }
    ensureTickerV8();
}
