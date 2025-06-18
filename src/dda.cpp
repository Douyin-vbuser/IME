#include <jni.h>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cstdint>
#include <limits>
#include <array>

#ifdef __SSE2__
#include <emmintrin.h>
#endif

#ifdef __AVX2__
#include <immintrin.h>
#endif

#define MAX_CACHE_SIZE 2048
#define MAX_BLOCKS_PASSED 5
#define BLOCK_HASH_SIZE 16384

// 快速倒数平方根
inline double fastInvSqrt(double x) {
    double y = x;
    double x2 = y * 0.5;
    int64_t i = *(int64_t*)&y;
    i = 0x5fe6eb50c7b537a9 - (i >> 1);
    y = *(double*)&i;
    y = y * (1.5 - (x2 * y * y));
    return y;
}

// 优化的方块位置结构
struct alignas(8) BlockPosition {
    int32_t x : 26;
    int32_t y : 12;
    int32_t z : 26;

    BlockPosition(int32_t _x, int32_t _y, int32_t _z) : x(_x), y(_y), z(_z) {}

    bool operator==(const BlockPosition& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// 自定义哈希表实现
class BlockHashTable {
private:
    struct Entry {
        uint64_t key;
        bool occupied;
    };

    std::array<Entry, BLOCK_HASH_SIZE> table;
    size_t size_;

public:
    BlockHashTable() : size_(0) {
        clear();
    }

    void clear() {
        for(auto& entry : table) {
            entry.occupied = false;
        }
        size_ = 0;
    }

    void insert(uint64_t key) {
        size_t idx = key % BLOCK_HASH_SIZE;
        while(table[idx].occupied && table[idx].key != key) {
            idx = (idx + 1) % BLOCK_HASH_SIZE;
        }
        if(!table[idx].occupied) {
            table[idx].key = key;
            table[idx].occupied = true;
            size_++;
        }
    }

    bool contains(uint64_t key) const {
        size_t idx = key % BLOCK_HASH_SIZE;
        while(table[idx].occupied) {
            if(table[idx].key == key) return true;
            idx = (idx + 1) % BLOCK_HASH_SIZE;
        }
        return false;
    }

    size_t size() const { return size_; }
};

// 全局方块哈希表
static BlockHashTable blockTable;

// 优化的缓存实现
class alignas(64) VisibilityCache {
private:
    struct alignas(32) CacheEntry {
        double fromX, fromY, fromZ;
        double toX, toY, toZ;
        bool value;
        bool valid;
    };

    static constexpr size_t CACHE_SIZE = MAX_CACHE_SIZE;
    std::array<CacheEntry, CACHE_SIZE> cache;
    size_t index;

    static inline size_t hashCoords(double x, double y, double z) {
        const uint64_t prime = 1099511628211ULL;
        uint64_t hash = 14695981039346656037ULL;
        hash = (hash ^ *reinterpret_cast<uint64_t*>(&x)) * prime;
        hash = (hash ^ *reinterpret_cast<uint64_t*>(&y)) * prime;
        hash = (hash ^ *reinterpret_cast<uint64_t*>(&z)) * prime;
        return hash % CACHE_SIZE;
    }

public:
    VisibilityCache() : index(0) {
        clear();
    }

    void clear() {
        for(auto& entry : cache) {
            entry.valid = false;
        }
        index = 0;
    }

    bool get(double fromX, double fromY, double fromZ,
             double toX, double toY, double toZ, bool& result) {
        size_t h = hashCoords(fromX + toX, fromY + toY, fromZ + toZ);
        const CacheEntry& entry = cache[h];

        if(entry.valid &&
           entry.fromX == fromX && entry.fromY == fromY && entry.fromZ == fromZ &&
           entry.toX == toX && entry.toY == toY && entry.toZ == toZ) {
            result = entry.value;
            return true;
        }
        return false;
    }

    void put(double fromX, double fromY, double fromZ,
             double toX, double toY, double toZ, bool value) {
        size_t h = hashCoords(fromX + toX, fromY + toY, fromZ + toZ);
        CacheEntry& entry = cache[h];

        entry.fromX = fromX;
        entry.fromY = fromY;
        entry.fromZ = fromZ;
        entry.toX = toX;
        entry.toY = toY;
        entry.toZ = toZ;
        entry.value = value;
        entry.valid = true;
    }
};

static VisibilityCache visibilityCache;

// 优化的DDA实现
bool checkVisibilityDDA(double fromX, double fromY, double fromZ,
                       double toX, double toY, double toZ) {
    // 快速距离检查
    const double dx = toX - fromX;
    const double dy = toY - fromY;
    const double dz = toZ - fromZ;

    const double distSq = dx*dx + dy*dy + dz*dz;
    if(distSq < 1e-8) return true;

    // 使用快速倒数算法
    const double invDist = fastInvSqrt(distSq);
    const double vx = dx * invDist;
    const double vy = dy * invDist;
    const double vz = dz * invDist;

    // 优化的步进方向计算
    const int stepX = (vx >= 0) - (vx < 0);
    const int stepY = (vy >= 0) - (vy < 0);
    const int stepZ = (vz >= 0) - (vz < 0);

    // 预计算步进增量
    const double tDeltaX = std::abs(invDist / vx);
    const double tDeltaY = std::abs(invDist / vy);
    const double tDeltaZ = std::abs(invDist / vz);

    // 优化的初始t值计算
    double tMaxX = ((stepX > 0 ? std::ceil(fromX) - fromX : fromX - std::floor(fromX)) * invDist) / std::abs(vx);
    double tMaxY = ((stepY > 0 ? std::ceil(fromY) - fromY : fromY - std::floor(fromY)) * invDist) / std::abs(vy);
    double tMaxZ = ((stepZ > 0 ? std::ceil(fromZ) - fromZ : fromZ - std::floor(fromZ)) * invDist) / std::abs(vz);

    if(!std::isfinite(tMaxX)) tMaxX = std::numeric_limits<double>::max();
    if(!std::isfinite(tMaxY)) tMaxY = std::numeric_limits<double>::max();
    if(!std::isfinite(tMaxZ)) tMaxZ = std::numeric_limits<double>::max();

    // 目标坐标
    const int targetX = static_cast<int>(std::floor(toX));
    const int targetY = static_cast<int>(std::floor(toY));
    const int targetZ = static_cast<int>(std::floor(toZ));

    // 当前位置
    int x = static_cast<int>(std::floor(fromX));
    int y = static_cast<int>(std::floor(fromY));
    int z = static_cast<int>(std::floor(fromZ));

    int blocksPassed = 0;
    const double maxTravelSq = distSq + 1.0;

    while(true) {
        // 检查是否到达目标
        if(x == targetX && y == targetY && z == targetZ) {
            return true;
        }

        // 距离检查
        const double dx2 = x - fromX;
        const double dy2 = y - fromY;
        const double dz2 = z - fromZ;
        if(dx2*dx2 + dy2*dy2 + dz2*dz2 > maxTravelSq) {
            return false;
        }

        // 优化的方块检查
        const uint64_t pos = (static_cast<uint64_t>(x) << 40) |
                            (static_cast<uint64_t>(y) << 20) |
                             static_cast<uint64_t>(z);

        if(blockTable.contains(pos)) {
            if(++blocksPassed > MAX_BLOCKS_PASSED) {
                return false;
            }
        }

        // 优化的步进逻辑
        if(tMaxX < tMaxY) {
            if(tMaxX < tMaxZ) {
                x += stepX;
                tMaxX += tDeltaX;
            } else {
                z += stepZ;
                tMaxZ += tDeltaZ;
            }
        } else {
            if(tMaxY < tMaxZ) {
                y += stepY;
                tMaxY += tDeltaY;
            } else {
                z += stepZ;
                tMaxZ += tDeltaZ;
            }
        }
    }
}

extern "C" {

JNIEXPORT void JNICALL Java_com_vbuser_particulate_render_NativeBlockRenderer_updateMap(
    JNIEnv* env, jclass, jlongArray positions) {

    jsize len = env->GetArrayLength(positions);
    jlong* arr = env->GetLongArrayElements(positions, nullptr);

    blockTable.clear();

    for(jsize i = 0; i < len; ++i) {
        blockTable.insert(static_cast<uint64_t>(arr[i]));
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

    bool result;
    if(visibilityCache.get(fromX, fromY, fromZ, toX, toY, toZ, result)) {
        return static_cast<jboolean>(result);
    }

    result = checkVisibilityDDA(fromX, fromY, fromZ, toX, toY, toZ);
    visibilityCache.put(fromX, fromY, fromZ, toX, toY, toZ, result);
    return static_cast<jboolean>(result);
}

}