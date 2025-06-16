#include <jni.h>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <limits>
// 移除 immintrin.h 包含

#define MAX_CACHE_SIZE 2000
#define MAX_BLOCKS_PASSED 5

// 优化后的全局方块位置集合
std::unordered_set<uint64_t> blockPositions;

// 高性能缓存结构
class VisibilityCache {
public:
    struct Key {
        int from[3];
        int to[3];
        
        bool operator==(const Key& other) const {
            return from[0] == other.from[0] &&
                   from[1] == other.from[1] &&
                   from[2] == other.from[2] &&
                   to[0] == other.to[0] &&
                   to[1] == other.to[1] &&
                   to[2] == other.to[2];
        }
    };
    
    struct Hash {
        size_t operator()(const Key& k) const {
            // 优化哈希计算 - 使用位混合
            size_t h = 0;
            h ^= (static_cast<size_t>(k.from[0]) << 32 | k.from[1];
            h ^= (static_cast<size_t>(k.from[2]) << 16 | k.to[0];
            h ^= (static_cast<size_t>(k.to[1]) << 32 | k.to[2];
            return h;
        }
    };
    
    struct CacheEntry {
        Key key;
        bool value;
    };
    
    std::vector<CacheEntry> cache;
    size_t capacity;
    size_t index = 0;
    
    VisibilityCache(size_t cap) : capacity(cap) {
        cache.resize(cap);
    }
    
    bool get(const Key& key, bool& result) {
        for (size_t i = 0; i < capacity; ++i) {
            if (cache[i].key == key) {
                result = cache[i].value;
                return true;
            }
        }
        return false;
    }
    
    void put(const Key& key, bool value) {
        cache[index] = {key, value};
        index = (index + 1) % capacity;
    }
    
    void clear() {
        std::fill(cache.begin(), cache.end(), CacheEntry());
        index = 0;
    }
};

VisibilityCache visibilityCache(MAX_CACHE_SIZE);

// 高度优化的DDA算法
__attribute__((always_inline)) 
bool checkVisibilityDDA(double fromX, double fromY, double fromZ, 
                        double toX, double toY, double toZ) {
    const double dx = toX - fromX;
    const double dy = toY - fromY;
    const double dz = toZ - fromZ;
    
    // 使用平方距离避免开方
    const double distSq = dx*dx + dy*dy + dz*dz;
    if (distSq < 1e-8) return true;
    
    const double invDist = 1.0 / std::sqrt(distSq);
    const double vx = dx * invDist;
    const double vy = dy * invDist;
    const double vz = dz * invDist;
    
    // 预计算步进方向
    const int stepX = vx >= 0 ? 1 : -1;
    const int stepY = vy >= 0 ? 1 : -1;
    const int stepZ = vz >= 0 ? 1 : -1;
    
    // 预计算步进增量
    const double tDeltaX = std::abs(1.0 / vx);
    const double tDeltaY = std::abs(1.0 / vy);
    const double tDeltaZ = std::abs(1.0 / vz);
    
    // 初始最大t值
    double tMaxX = vx != 0 ? (stepX > 0 ? std::ceil(fromX) - fromX : fromX - std::floor(fromX)) / std::abs(vx) 
                           : std::numeric_limits<double>::max();
    double tMaxY = vy != 0 ? (stepY > 0 ? std::ceil(fromY) - fromY : fromY - std::floor(fromY)) / std::abs(vy) 
                           : std::numeric_limits<double>::max();
    double tMaxZ = vz != 0 ? (stepZ > 0 ? std::ceil(fromZ) - fromZ : fromZ - std::floor(fromZ)) / std::abs(vz) 
                           : std::numeric_limits<double>::max();
    
    // 目标整数坐标
    const int targetX = static_cast<int>(std::floor(toX));
    const int targetY = static_cast<int>(std::floor(toY));
    const int targetZ = static_cast<int>(std::floor(toZ));
    
    // 当前坐标
    double x = fromX;
    double y = fromY;
    double z = fromZ;
    
    int blocksPassed = 0;
    const double eps = 1e-6;
    const double maxTravelSq = distSq + 2.0; // 容差
    
    // 提前计算位置编码
    uint64_t lastPos = 0;
    int lastX = 0, lastY = 0, lastZ = 0;
    
    while (true) {
        // 整数坐标转换
        const int ix = static_cast<int>(std::floor(x + eps));
        const int iy = static_cast<int>(std::floor(y + eps));
        const int iz = static_cast<int>(std::floor(z + eps));
        
        // 检查是否到达目标
        if (ix == targetX && iy == targetY && iz == targetZ) {
            return true;
        }
        
        // 增量距离检查
        const double dx2 = x - fromX;
        const double dy2 = y - fromY;
        const double dz2 = z - fromZ;
        const double traveledSq = dx2*dx2 + dy2*dy2 + dz2*dz2;
        
        if (traveledSq > maxTravelSq) {
            return false;
        }
        
        // 位置编码优化 - 避免重复计算
        uint64_t currentPos = 0;
        if (ix != lastX || iy != lastY || iz != lastZ) {
            currentPos = (static_cast<uint64_t>(ix) & 0x3FFFFFF) << 38 |
                         (static_cast<uint64_t>(iy) & 0xFFF) << 26 |
                         (static_cast<uint64_t>(iz) & 0x3FFFFFF);
            lastX = ix;
            lastY = iy;
            lastZ = iz;
            lastPos = currentPos;
        } else {
            currentPos = lastPos;
        }
        
        // 检查方块存在性
        if (blockPositions.find(currentPos) != blockPositions.end()) {
            if (++blocksPassed > MAX_BLOCKS_PASSED) {
                return false;
            }
        }
        
        // 无分支步进选择
        const bool xMin = (tMaxX <= tMaxY) && (tMaxX <= tMaxZ);
        const bool yMin = (tMaxY <= tMaxX) && (tMaxY <= tMaxZ);
        const bool zMin = (tMaxZ <= tMaxX) && (tMaxZ <= tMaxY);
        
        if (xMin) {
            x += stepX;
            tMaxX += tDeltaX;
        } else if (yMin) {
            y += stepY;
            tMaxY += tDeltaY;
        } else if (zMin) {
            z += stepZ;
            tMaxZ += tDeltaZ;
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
    blockPositions.reserve(len);
    
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
    
    // 使用整数坐标减少浮点精度问题
    VisibilityCache::Key key = {
        {static_cast<int>(fromX * 1000), static_cast<int>(fromY * 1000), static_cast<int>(fromZ * 1000)},
        {static_cast<int>(toX * 1000), static_cast<int>(toY * 1000), static_cast<int>(toZ * 1000)}
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