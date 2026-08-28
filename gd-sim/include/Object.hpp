#pragma once
#include <util.hpp>
#include <unordered_map>
#include <cstring>
#include <optional>
#include <cstddef>

struct ObjectContainer;
struct Player;

struct Object : public Entity {
    /// NOT object id. A unique ID associated with each Object for direct comparisons
    int id;

    /**
     * In GD, some objects have their collision checks later than others (blocks, hazards).
     * A higher prio numbers means collisions are processed later.
     */
    int prio;

    Object() = default;
    Object(Vec2D size, std::unordered_map<int, std::string>&& fields);

    /// Determines if the object should be counted as colliding with the player.
    virtual bool touching(Player const&) const;

    /// Where all of the collision magic happens.
    virtual void collide(Player&) const;

    /// Create an object from a given level string mapping.
    static std::optional<ObjectContainer> create(std::unordered_map<int, std::string>&& ob);
};

/**
 * Stores polymorphic simulator objects inline to avoid a heap allocation per level object.
 * Modern 2.2 objects carry more setup state than the old eight-byte padding allowed, so
 * keep a larger aligned payload while preserving the same contiguous-storage design.
 */
struct ObjectContainer {
    static constexpr size_t extraStorage = 0x80;
    alignas(std::max_align_t) std::byte buffer[sizeof(Object) + extraStorage]{};

    ObjectContainer(ObjectContainer& cont) { std::memcpy(buffer, cont.buffer, sizeof(buffer)); }
    ObjectContainer(ObjectContainer const& cont) { std::memcpy(buffer, cont.buffer, sizeof(buffer)); }

    template <class T>
    ObjectContainer(T&& obj) {
        static_assert(sizeof(T) <= sizeof(buffer), "Object subclass exceeds ObjectContainer storage");
        std::memcpy(buffer, static_cast<void const*>(&obj), sizeof(T));
    }

    Object const* operator->() const {
        return reinterpret_cast<Object const*>(buffer);
    }
    Object* operator->() {
        return reinterpret_cast<Object*>(buffer);
    }
};
