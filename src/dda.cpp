#include <jni.h>
#include <unordered_set>
#include <list>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <limits>
#include <cstdint>

#define MAX_CACHE_SIZE 1000
#define MAX_BLOCKS_PASSED 5

// 全局方块位置集合
std::unordered_set<uint64_t> blockPositions;

// 可见性缓存结构
class VisibilityCache {
public:
    struct Key {
        double from[3];
        double to[3];

        bool operator==(const Key& other) const {
            return std::memcmp(this, &other, sizeof(Key)) == 0;
        }
    };

    struct Hash {
        size_t operator()(const Key& k) const {
            size_t h = 0;
            for (double d : k.from)
                h ^= std::hash<double>()(d) + 0x9e3779b9 + (h<<6) + (h>>2);
            for (double d : k.to)
                h ^= std::hash<double>()(d) + 0x9e3779b9 + (h<<6) + (h>>2);
            return h;
        }
    };

    typedef std::pair<Key, bool> CacheItem;
    typedef std::list<CacheItem> ItemList;
    typedef std::unordered_map<Key, typename ItemList::iterator, Hash> ItemMap;

    ItemList lruList;
    ItemMap cacheMap;
    size_t capacity;

    VisibilityCache(size_t cap) : capacity(cap) {}

    bool get(const Key& key, bool& result) {
        auto it = cacheMap.find(key);
        if (it == cacheMap.end()) return false;

        lruList.splice(lruList.begin(), lruList, it->second);
        result = it->second->second;
        return true;
    }

    void put(const Key& key, bool value) {
        auto it = cacheMap.find(key);
        if (it != cacheMap.end()) {
            it->second->second = value;
            lruList.splice(lruList.begin(), lruList, it->second);
            return;
        }

        if (cacheMap.size() >= capacity) {
            auto last = lruList.end();
            last--;
            cacheMap.erase(last->first);
            lruList.pop_back();
        }

        lruList.push_front({key, value});
        cacheMap[key] = lruList.begin();
    }

    void clear() {
        lruList.clear();
        cacheMap.clear();
    }
};

VisibilityCache visibilityCache(MAX_CACHE_SIZE);

// DDA算法实现
bool checkVisibilityDDA(double fromX, double fromY, double fromZ,
                        double toX, double toY, double toZ) {
    const double dx = toX - fromX;
    const double dy = toY - fromY;
    const double dz = toZ - fromZ;

    const double distSq = dx*dx + dy*dy + dz*dz;
    if (distSq < 1e-8) return true;

    const double dist = std::sqrt(distSq);
    const double invDist = 1.0 / dist;
    const double vx = dx * invDist;
    const double vy = dy * invDist;
    const double vz = dz * invDist;

    double x = fromX;
    double y = fromY;
    double z = fromZ;

    const int stepX = vx >= 0 ? 1 : -1;
    const int stepY = vy >= 0 ? 1 : -1;
    const int stepZ = vz >= 0 ? 1 : -1;

    const double tDeltaX = std::abs(1.0 / vx);
    const double tDeltaY = std::abs(1.0 / vy);
    const double tDeltaZ = std::abs(1.0 / vz);

    double tMaxX = vx != 0 ? (stepX > 0 ? std::ceil(fromX) - fromX : fromX - std::floor(fromX)) / std::abs(vx)
                           : std::numeric_limits<double>::max();
    double tMaxY = vy != 0 ? (stepY > 0 ? std::ceil(fromY) - fromY : fromY - std::floor(fromY)) / std::abs(vy)
                           : std::numeric_limits<double>::max();
    double tMaxZ = vz != 0 ? (stepZ > 0 ? std::ceil(fromZ) - fromZ : fromZ - std::floor(fromZ)) / std::abs(vz)
                           : std::numeric_limits<double>::max();

    const int targetX = static_cast<int>(std::floor(toX));
    const int targetY = static_cast<int>(std::floor(toY));
    const int targetZ = static_cast<int>(std::floor(toZ));

    int blocksPassed = 0;
    const double eps = 1e-6;

    while (true) {
        const int ix = static_cast<int>(std::floor(x + eps));
        const int iy = static_cast<int>(std::floor(y + eps));
        const int iz = static_cast<int>(std::floor(z + eps));

        // 到达目标位置
        if (ix == targetX && iy == targetY && iz == targetZ) {
            return true;
        }

        // 检查当前方块是否在渲染列表中
        const uint64_t pos = ((static_cast<uint64_t>(ix) & 0x3FFFFFF) << 38 |
                             ((static_cast<uint64_t>(iy) & 0xFFF) << 26 |
                             (static_cast<uint64_t>(iz) & 0x3FFFFFF)));

        if (blockPositions.find(pos) != blockPositions.end()) {
            if (++blocksPassed > MAX_BLOCKS_PASSED) {
                return false;
            }
        }

        // 选择下一个步进方向
        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) {
                x += stepX;
                tMaxX += tDeltaX;
            } else {
                z += stepZ;
                tMaxZ += tDeltaZ;
            }
        } else {
            if (tMaxY < tMaxZ) {
                y += stepY;
                tMaxY += tDeltaY;
            } else {
                z += stepZ;
                tMaxZ += tDeltaZ;
            }
        }

        // 检查是否超出距离
        const double dx2 = x - fromX;
        const double dy2 = y - fromY;
        const double dz2 = z - fromZ;
        if (dx2*dx2 + dy2*dy2 + dz2*dz2 > distSq + 1.0) {
            return false;
        }
    }
}

// JNI方法实现
extern "C" {

JNIEXPORT void JNICALL Java_com_vbuser_particulate_render_NativeBlockRenderer_updateMap(
    JNIEnv* env, jclass, jlongArray positions) {
    jsize len = env->GetArrayLength(positions);
    jlong* arr = env->GetLongArrayElements(positions, nullptr);

    blockPositions.clear();
    for (jsize i = 0; i < len; ++i) {
        blockPositions.insert(static_cast<uint64_t>(arr[i]));
    }

    env->ReleaseLongArrayElements(positions, arr, JNI_ABORT);
}

JNIEXPORT void JNICALL Java_com_vbuser_particulate_render_NativeBlockRenderer_clearVisibilityCache(
    JNIEnv*, jclass) {
    visibilityCache.clear();
}

JNIEXPORT jboolean JNICALL Java_com_vbuser_particulate_render_NativeBlockRenderer_visibleInMap(
    JNIEnv*, jclass,
    jdouble fromX, jdouble fromY, jdouble fromZ,
    jdouble toX, jdouble toY, jdouble toZ) {

    VisibilityCache::Key key = {
        {fromX, fromY, fromZ},
        {toX, toY, toZ}
    };

    bool result;
    if (visibilityCache.get(key, result)) {
        return static_cast<jboolean>(result);
    }

    result = checkVisibilityDDA(fromX, fromY, fromZ, toX, toY, toZ);
    visibilityCache.put(key, result);
    return static_cast<jboolean>(result);
}

}
