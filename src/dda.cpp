#include <jni.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <memory>
#include <array>
#include <mutex>
#include <climits>  // 添加这个头文件

#ifdef __SSE4_1__
#include <smmintrin.h>
#endif

#ifdef __AVX2__
#include <immintrin.h>
#endif

// 配置参数
constexpr int GRID_SIZE = 16;
constexpr int OCTREE_THRESHOLD = 32;
constexpr int MAX_CACHE_SIZE = 4096;
constexpr int MAX_BLOCKS_PASSED = 5;

// 方块位置结构
struct BlockPosition {
    int32_t x, y, z;

    BlockPosition(int32_t x, int32_t y, int32_t z)
        : x(x), y(y), z(z) {}

    bool operator==(const BlockPosition& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    uint64_t toLong() const {
        return ((static_cast<uint64_t>(x) & 0x3FFFFFF) << 38 |
               ((static_cast<uint64_t>(z) & 0x3FFFFFF) << 12 |
               (static_cast<uint64_t>(y) & 0xFFF)));
    }
};

// 八叉树节点
class OctreeNode {
public:
    enum { LEAF = 0, BRANCH = 1 };

    int type;
    int minX, minY, minZ;
    int maxX, maxY, maxZ;

    // 分支节点
    std::unique_ptr<OctreeNode> children[8];

    // 叶子节点
    std::vector<BlockPosition> blocks;

    OctreeNode(int minX, int minY, int minZ, int maxX, int maxY, int maxZ)
        : type(LEAF), minX(minX), minY(minY),
          minZ(minZ), maxX(maxX), maxY(maxY), maxZ(maxZ) {}

    // 插入方块
    void insert(const BlockPosition& block);

    // 分割节点
    void split();

    // 检查视线是否与节点相交
    bool intersectsRay(double fromX, double fromY, double fromZ,
                      double dx, double dy, double dz,
                      double invDx, double invDy, double invDz,
                      double& tMin, double& tMax) const {
        double tx1 = (minX - fromX) * invDx;
        double tx2 = (maxX - fromX) * invDx;

        tMin = std::min(tx1, tx2);
        tMax = std::max(tx1, tx2);

        double ty1 = (minY - fromY) * invDy;
        double ty2 = (maxY - fromY) * invDy;

        tMin = std::max(tMin, std::min(ty1, ty2));
        tMax = std::min(tMax, std::max(ty1, ty2));

        double tz1 = (minZ - fromZ) * invDz;
        double tz2 = (maxZ - fromZ) * invDz;

        tMin = std::max(tMin, std::min(tz1, tz2));
        tMax = std::min(tMax, std::max(tz1, tz2));

        return tMax >= tMin && tMin <= 1.0 && tMax >= 0.0;
    }
};

void OctreeNode::insert(const BlockPosition &block) {
    // 叶子节点且未达阈值
    if (type == LEAF && blocks.size() < OCTREE_THRESHOLD) {
        blocks.push_back(block);
        return;
    }

    // 需要转换为分支节点
    if (type == LEAF) {
        split();
    }

    // 插入到合适的子节点
    int midX = (minX + maxX) / 2;
    int midY = (minY + maxY) / 2;
    int midZ = (minZ + maxZ) / 2;

    int index = 0;
    if (block.x >= midX) index |= 1;
    if (block.y >= midY) index |= 2;
    if (block.z >= midZ) index |= 4;

    children[index]->insert(block);
}

void OctreeNode::split() {
    int midX = (minX + maxX) / 2;
    int midY = (minY + maxY) / 2;
    int midZ = (minZ + maxZ) / 2;

    // 创建子节点
    children[0] = std::make_unique<OctreeNode>(minX, minY, minZ, midX, midY, midZ);
    children[1] = std::make_unique<OctreeNode>(midX, minY, minZ, maxX, midY, midZ);
    children[2] = std::make_unique<OctreeNode>(minX, midY, minZ, midX, maxY, midZ);
    children[3] = std::make_unique<OctreeNode>(midX, midY, minZ, maxX, maxY, midZ);
    children[4] = std::make_unique<OctreeNode>(minX, minY, midZ, midX, midY, maxZ);
    children[5] = std::make_unique<OctreeNode>(midX, minY, midZ, maxX, midY, maxZ);
    children[6] = std::make_unique<OctreeNode>(minX, midY, midZ, midX, maxY, maxZ);
    children[7] = std::make_unique<OctreeNode>(midX, midY, midZ, maxX, maxY, maxZ);

    // 移动现有方块到子节点
    for (const auto& block : blocks) {
        int index = 0;
        if (block.x >= midX) index |= 1;
        if (block.y >= midY) index |= 2;
        if (block.z >= midZ) index |= 4;
        children[index]->insert(block);
    }

    blocks.clear();
    type = BRANCH;
}

// 优化的八叉树管理器
class BlockOctree {
private:
    std::unique_ptr<OctreeNode> root;
    std::mutex treeMutex;

public:
    void build(const std::vector<BlockPosition>& blocks) {
        if (blocks.empty()) {
            root.reset();
            return;
        }

        // 计算边界
        int minX = INT_MAX, minY = INT_MAX, minZ = INT_MAX;
        int maxX = INT_MIN, maxY = INT_MIN, maxZ = INT_MIN;

        for (const auto& block : blocks) {
            minX = std::min(minX, block.x);
            minY = std::min(minY, block.y);
            minZ = std::min(minZ, block.z);
            maxX = std::max(maxX, block.x + 1);
            maxY = std::max(maxY, block.y + 1);
            maxZ = std::max(maxZ, block.z + 1);
        }

        // 扩展边界到2的幂
        int size = 1;
        // 修复std::max初始化列表问题
        int maxDim = std::max(maxX - minX, std::max(maxY - minY, maxZ - minZ));
        while (size < maxDim) size <<= 1;

        maxX = minX + size;
        maxY = minY + size;
        maxZ = minZ + size;

        // 构建八叉树
        std::lock_guard<std::mutex> lock(treeMutex);
        root = std::make_unique<OctreeNode>(minX, minY, minZ, maxX, maxY, maxZ);

        for (const auto& block : blocks) {
            root->insert(block);
        }
    }

    bool checkVisibility(double fromX, double fromY, double fromZ,
                        double toX, double toY, double toZ) {
        if (!root) return true;

        const double dx = toX - fromX;
        const double dy = toY - fromY;
        const double dz = toZ - fromZ;
        const double distSq = dx*dx + dy*dy + dz*dz;
        if (distSq < 1e-8) return true;

        // 方向向量
        const double invDist = 1.0 / std::sqrt(distSq);
        const double vx = dx * invDist;
        const double vy = dy * invDist;
        const double vz = dz * invDist;

        // 方向倒数（避免除0）
        const double invDx = (std::abs(vx) > 1e-8) ? 1.0 / vx : std::copysign(1e10, vx);
        const double invDy = (std::abs(vy) > 1e-8) ? 1.0 / vy : std::copysign(1e10, vy);
        const double invDz = (std::abs(vz) > 1e-8) ? 1.0 / vz : std::copysign(1e10, vz);

        std::lock_guard<std::mutex> lock(treeMutex);
        return traverseOctree(*root, fromX, fromY, fromZ, vx, vy, vz,
                            invDx, invDy, invDz, toX, toY, toZ, distSq);
    }

private:
    static bool traverseOctree(const OctreeNode& node,
                               double fromX, double fromY, double fromZ,
                               double vx, double vy, double vz,
                               double invDx, double invDy, double invDz,
                               double toX, double toY, double toZ,
                               double distSq);

    static void calculateTraversalOrder(int order[8],
                                        double fromX, double fromY, double fromZ,
                                        double vx, double vy, double vz,
                                        int minX, int minY, int minZ,
                                        int maxX, int maxY, int maxZ) {

        // 确定进入方向
        int startX = (vx >= 0) ? 0 : 1;
        int startY = (vy >= 0) ? 0 : 1;
        int startZ = (vz >= 0) ? 0 : 1;

        // 使用Morton码排序
        order[0] = (startX << 0) | (startY << 1) | (startZ << 2);
        order[1] = (startX << 0) | (startY << 1) | ((1 - startZ) << 2);
        order[2] = (startX << 0) | ((1 - startY) << 1) | (startZ << 2);
        order[3] = (startX << 0) | ((1 - startY) << 1) | ((1 - startZ) << 2);
        order[4] = ((1 - startX) << 0) | (startY << 1) | (startZ << 2);
        order[5] = ((1 - startX) << 0) | (startY << 1) | ((1 - startZ) << 2);
        order[6] = ((1 - startX) << 0) | ((1 - startY) << 1) | (startZ << 2);
        order[7] = ((1 - startX) << 0) | ((1 - startY) << 1) | ((1 - startZ) << 2);
    }

    static bool checkBlocksInNode(const OctreeNode& node,
                                  double fromX, double fromY, double fromZ,
                                  double toX, double toY, double toZ,
                                  double distSq) {
        // 简化的DDA检查节点内的方块
        const double dx = toX - fromX;
        const double dy = toY - fromY;
        const double dz = toZ - fromZ;
        const double invDist = 1.0 / std::sqrt(dx*dx + dy*dy + dz*dz);
        const double vx = dx * invDist;
        const double vy = dy * invDist;
        const double vz = dz * invDist;

        int stepX = (vx >= 0) - (vx < 0);
        int stepY = (vy >= 0) - (vy < 0);
        int stepZ = (vz >= 0) - (vz < 0);

        double tDeltaX = std::abs(invDist / vx);
        double tDeltaY = std::abs(invDist / vy);
        double tDeltaZ = std::abs(invDist / vz);

        double tMaxX = ((stepX > 0 ? std::ceil(fromX) - fromX : fromX - std::floor(fromX)) * invDist) / std::abs(vx);
        double tMaxY = ((stepY > 0 ? std::ceil(fromY) - fromY : fromY - std::floor(fromY)) * invDist) / std::abs(vy);
        double tMaxZ = ((stepZ > 0 ? std::ceil(fromZ) - fromZ : fromZ - std::floor(fromZ)) * invDist) / std::abs(vz);

        if (!std::isfinite(tMaxX)) tMaxX = std::numeric_limits<double>::max();
        if (!std::isfinite(tMaxY)) tMaxY = std::numeric_limits<double>::max();
        if (!std::isfinite(tMaxZ)) tMaxZ = std::numeric_limits<double>::max();

        const int targetX = static_cast<int>(std::floor(toX));
        const int targetY = static_cast<int>(std::floor(toY));
        const int targetZ = static_cast<int>(std::floor(toZ));

        int x = static_cast<int>(std::floor(fromX));
        int y = static_cast<int>(std::floor(fromY));
        int z = static_cast<int>(std::floor(fromZ));

        int blocksPassed = 0;
        const double maxTravelSq = distSq + 1.0;

        while (true) {
            if (x == targetX && y == targetY && z == targetZ) {
                return true;
            }

            // 距离检查
            const double dx2 = x - fromX;
            const double dy2 = y - fromY;
            const double dz2 = z - fromZ;
            if (dx2*dx2 + dy2*dy2 + dz2*dz2 > maxTravelSq) {
                return true;
            }

            // 检查当前方块是否在节点内
            BlockPosition current(x, y, z);
            auto it = std::find(node.blocks.begin(), node.blocks.end(), current);
            if (it != node.blocks.end()) {
                if (++blocksPassed > MAX_BLOCKS_PASSED) {
                    return false;
                }
            }

            // 步进逻辑
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
        }
    }
};

bool BlockOctree::traverseOctree(const OctreeNode &node, double fromX, double fromY, double fromZ, double vx, double vy,
    double vz, double invDx, double invDy, double invDz, double toX, double toY, double toZ, double distSq) {
    // 检查节点是否与光线相交
    double tMin, tMax;
    if (!node.intersectsRay(fromX, fromY, fromZ, vx, vy, vz,
                            invDx, invDy, invDz, tMin, tMax)) {
        return true;
    }

    // 叶子节点处理
    if (node.type == OctreeNode::LEAF) {
        return checkBlocksInNode(node, fromX, fromY, fromZ,
                                 toX, toY, toZ, distSq);
    }

    // 分支节点处理
    int visitOrder[8];
    calculateTraversalOrder(visitOrder, fromX, fromY, fromZ, vx, vy, vz,
                            node.minX, node.minY, node.minZ,
                            node.maxX, node.maxY, node.maxZ);

    // 递归检查子节点
    for (int idx : visitOrder) {
        if (node.children[idx]) {
            if (!traverseOctree(*node.children[idx], fromX, fromY, fromZ,
                                vx, vy, vz, invDx, invDy, invDz,
                                toX, toY, toZ, distSq)) {
                return false;
            }
        }
    }

    return true;
}

// 全局八叉树实例
static BlockOctree blockOctree;
static std::vector<BlockPosition> currentBlocks;
static std::mutex blocksMutex;

// 优化的缓存系统
class VisibilityCache {
private:
    struct alignas(64) CacheEntry {
        double fromX, fromY, fromZ;
        double toX, toY, toZ;
        bool value;
        bool valid;
        uint64_t hash;
    };

    static constexpr size_t CACHE_SIZE = MAX_CACHE_SIZE;
    std::array<CacheEntry, CACHE_SIZE> cache{};
    size_t index;

    static inline uint64_t spatialHash(double x, double y, double z) {
        const auto px = static_cast<uint64_t>(x * 1000);
        const auto py = static_cast<uint64_t>(y * 1000);
        const auto pz = static_cast<uint64_t>(z * 1000);
        return (px * 73856093) ^ (py * 19349663) ^ (pz * 83492791);
    }

public:
    VisibilityCache() : index(0) {
        clear();
    }

    void clear() {
        for (auto& entry : cache) {
            entry.valid = false;
        }
        index = 0;
    }

    bool get(double fromX, double fromY, double fromZ,
             double toX, double toY, double toZ, bool& result) const {
        const uint64_t hash = spatialHash(fromX, fromY, fromZ) ^
                             spatialHash(toX, toY, toZ);
        const size_t idx = hash % CACHE_SIZE;

        if (cache[idx].valid && cache[idx].hash == hash) {
            result = cache[idx].value;
            return true;
        }
        return false;
    }

    void put(double fromX, double fromY, double fromZ,
             double toX, double toY, double toZ, bool value) {
        const uint64_t hash = spatialHash(fromX, fromY, fromZ) ^
                             spatialHash(toX, toY, toZ);
        const size_t idx = hash % CACHE_SIZE;

        cache[idx].fromX = fromX;
        cache[idx].fromY = fromY;
        cache[idx].fromZ = fromZ;
        cache[idx].toX = toX;
        cache[idx].toY = toY;
        cache[idx].toZ = toZ;
        cache[idx].value = value;
        cache[idx].valid = true;
        cache[idx].hash = hash;
    }
};

static VisibilityCache visibilityCache;

extern "C" {

JNIEXPORT void JNICALL Java_com_vbuser_particulate_render_NativeBlockRenderer_updateMap(
    JNIEnv* env, jclass, jlongArray positions) {

    jsize len = env->GetArrayLength(positions);
    jlong* arr = env->GetLongArrayElements(positions, nullptr);

    std::vector<BlockPosition> newBlocks;
    newBlocks.reserve(len);

    for (jsize i = 0; i < len; ++i) {
        auto posVal = static_cast<uint64_t>(arr[i]);
        int x = static_cast<int>((posVal >> 38) & 0x3FFFFFF);
        int z = static_cast<int>((posVal >> 12) & 0x3FFFFFF);
        int y = static_cast<int>(posVal & 0xFFF);
        newBlocks.emplace_back(x, y, z);
    }

    env->ReleaseLongArrayElements(positions, arr, JNI_ABORT);

    {
        std::lock_guard<std::mutex> lock(blocksMutex);
        currentBlocks = std::move(newBlocks);
        blockOctree.build(currentBlocks);
    }
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
    if (visibilityCache.get(fromX, fromY, fromZ, toX, toY, toZ, result)) {
        return static_cast<jboolean>(result);
    }

    {
        std::lock_guard<std::mutex> lock(blocksMutex);
        result = blockOctree.checkVisibility(
            fromX, fromY, fromZ, toX, toY, toZ);
    }

    visibilityCache.put(fromX, fromY, fromZ, toX, toY, toZ, result);
    return static_cast<jboolean>(result);
}

}