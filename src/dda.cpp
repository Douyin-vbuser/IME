#include <jni.h>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <list>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cstdint>
#include <limits>

#define MAX_CACHE_SIZE 2000
#define MAX_BLOCKS_PASSED 5

class VisibilityCache {
public:
    struct Key {
        int32_t coords[6];

        bool operator==(const Key& other) const {
            return memcmp(coords, other.coords, sizeof(coords)) == 0;
        }
    };

    struct KeyHash {
        std::size_t operator()(const Key& k) const {
            std::size_t h = 0;
            for (int i = 0; i < 6; ++i) {
                h = h * 31 + std::hash<int32_t>{}(k.coords[i]);
            }
            return h;
        }
    };

private:
    std::unordered_map<Key, bool, KeyHash> cacheMap;
    std::list<Key> lruList;
    std::unordered_map<Key, std::list<Key>::iterator, KeyHash> lruMap;
    size_t capacity;

public:
    explicit VisibilityCache(size_t cap) : capacity(cap) {
        cacheMap.reserve(cap);
        lruMap.reserve(cap);
    }

    bool get(const Key& key, bool& result) {
        auto it = cacheMap.find(key);
        if (it != cacheMap.end()) {
            auto lruIt = lruMap.find(key);
            if (lruIt != lruMap.end()) {
                lruList.erase(lruIt->second);
                lruList.push_front(key);
                lruIt->second = lruList.begin();
            }
            result = it->second;
            return true;
        }
        return false;
    }

    void put(const Key& key, bool value) {
        if (cacheMap.size() >= capacity && !lruList.empty()) {
            const Key& oldKey = lruList.back();
            cacheMap.erase(oldKey);
            lruMap.erase(oldKey);
            lruList.pop_back();
        }

        auto [it, inserted] = cacheMap.insert_or_assign(key, value);
        lruList.push_front(key);
        lruMap[key] = lruList.begin();
    }

    void clear() {
        cacheMap.clear();
        lruList.clear();
        lruMap.clear();
    }
};

std::unordered_set<uint64_t> blockPositions;
VisibilityCache visibilityCache(MAX_CACHE_SIZE);

bool checkVisibilityDDA(double fromX, double fromY, double fromZ,
                       double toX, double toY, double toZ) {
    const double dx = toX - fromX;
    const double dy = toY - fromY;
    const double dz = toZ - fromZ;

    const double distSq = dx*dx + dy*dy + dz*dz;
    if (distSq < 1e-8) return true;

    const double invDist = 1.0 / std::sqrt(distSq);
    const double vx = dx * invDist;
    const double vy = dy * invDist;
    const double vz = dz * invDist;

    const int stepX = vx >= 0 ? 1 : -1;
    const int stepY = vy >= 0 ? 1 : -1;
    const int stepZ = vz >= 0 ? 1 : -1;

    const double tDeltaX = std::abs(vx) > 1e-8 ? std::abs(1.0 / vx) : std::numeric_limits<double>::max();
    const double tDeltaY = std::abs(vy) > 1e-8 ? std::abs(1.0 / vy) : std::numeric_limits<double>::max();
    const double tDeltaZ = std::abs(vz) > 1e-8 ? std::abs(1.0 / vz) : std::numeric_limits<double>::max();

    double tMaxX = vx != 0 ? (stepX > 0 ? std::ceil(fromX) - fromX : fromX - std::floor(fromX)) / std::abs(vx)
                          : std::numeric_limits<double>::max();
    double tMaxY = vy != 0 ? (stepY > 0 ? std::ceil(fromY) - fromY : fromY - std::floor(fromY)) / std::abs(vy)
                          : std::numeric_limits<double>::max();
    double tMaxZ = vz != 0 ? (stepZ > 0 ? std::ceil(fromZ) - fromZ : fromZ - std::floor(fromZ)) / std::abs(vz)
                          : std::numeric_limits<double>::max();

    const int targetX = static_cast<int>(std::floor(toX));
    const int targetY = static_cast<int>(std::floor(toY));
    const int targetZ = static_cast<int>(std::floor(toZ));

    double x = fromX;
    double y = fromY;
    double z = fromZ;

    int blocksPassed = 0;
    const double eps = 1e-6;
    const double maxTravelSq = distSq + 2.0;

    while (true) {
        const int ix = static_cast<int>(std::floor(x + eps));
        const int iy = static_cast<int>(std::floor(y + eps));
        const int iz = static_cast<int>(std::floor(z + eps));

        if (ix == targetX && iy == targetY && iz == targetZ) {
            return true;
        }

        const double dx2 = x - fromX;
        const double dy2 = y - fromY;
        const double dz2 = z - fromZ;
        const double traveledSq = dx2*dx2 + dy2*dy2 + dz2*dz2;

        if (traveledSq > maxTravelSq) {
            return false;
        }

        uint64_t currentPos = (static_cast<uint64_t>(ix) & 0x3FFFFFF) << 38 |
                             (static_cast<uint64_t>(iy) & 0xFFF) << 26 |
                             (static_cast<uint64_t>(iz) & 0x3FFFFFF);

        if (blockPositions.find(currentPos) != blockPositions.end()) {
            if (++blocksPassed > MAX_BLOCKS_PASSED) {
                return false;
            }
        }

        if (tMaxX <= tMaxY && tMaxX <= tMaxZ) {
            x += stepX;
            tMaxX += tDeltaX;
        } else if (tMaxY <= tMaxZ) {
            y += stepY;
            tMaxY += tDeltaY;
        } else {
            z += stepZ;
            tMaxZ += tDeltaZ;
        }
    }
}

extern "C" {

JNIEXPORT void JNICALL Java_com_vbuser_particulate_render_NativeBlockRenderer_updateMap(
    JNIEnv* env, jclass, jlongArray positions) {
    if (!positions) return;

    jsize len = env->GetArrayLength(positions);
    if (len <= 0) {
        blockPositions.clear();
        return;
    }

    jlong* elements = env->GetLongArrayElements(positions, nullptr);
    if (!elements) return;

    blockPositions.clear();
    blockPositions.reserve(len);

    for (jsize i = 0; i < len; ++i) {
        blockPositions.insert(static_cast<uint64_t>(elements[i]));
    }

    env->ReleaseLongArrayElements(positions, elements, JNI_ABORT);
}

JNIEXPORT void JNICALL Java_com_vbuser_particulate_render_NativeBlockRenderer_clearVisibilityCache(
    JNIEnv*, jclass) {
    visibilityCache.clear();
}

JNIEXPORT jboolean JNICALL Java_com_vbuser_particulate_render_NativeBlockRenderer_visibleInMap(
    JNIEnv*, jclass,
    jdouble fromX, jdouble fromY, jdouble fromZ,
    jdouble toX, jdouble toY, jdouble toZ) {

    VisibilityCache::Key key = {{
        static_cast<int32_t>(fromX * 1000),
        static_cast<int32_t>(fromY * 1000),
        static_cast<int32_t>(fromZ * 1000),
        static_cast<int32_t>(toX * 1000),
        static_cast<int32_t>(toY * 1000),
        static_cast<int32_t>(toZ * 1000)
    }};

    bool result;
    if (visibilityCache.get(key, result)) {
        return static_cast<jboolean>(result);
    }

    result = checkVisibilityDDA(fromX, fromY, fromZ, toX, toY, toZ);
    visibilityCache.put(key, result);
    return static_cast<jboolean>(result);
}

}
