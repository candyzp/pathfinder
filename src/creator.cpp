#include <Geode/Geode.hpp>
#include <Geode/ui/GeodeUI.hpp>
#include <Geode/modify/CreatorLayer.hpp>
#include <UIBuilder.hpp>

using namespace geode::prelude;

struct PathfinderCreatorLayer : geode::Modify<PathfinderCreatorLayer, CreatorLayer> {
    bool init() {
        if (!CreatorLayer::init())
            return false;

        auto* menu = CCMenu::create();
        menu->setID("pathfinder-creator-menu");
        menu->setPosition(ccp(32.f, 32.f));
        addChild(menu, 20);

        auto button = Build<BasedButtonSprite>::create(
            CCSprite::create("pathfinder.png"_spr),
            BaseType::Circle,
            4,
            3
        ).scale(0.58f);
        button->setTopRelativeScale(1.35f);

        button.intoMenuItem([](CCMenuItemSpriteExtra*) {
                geode::openSettingsPopup(Mod::get());
            })
            .id("pathfinder-creator-settings-button")
            .parent(menu);

        return true;
    }
};
