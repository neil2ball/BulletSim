#ifndef SHAPE_DATA_H
#define SHAPE_DATA_H
#include <vector>
#include <array>
#include <unordered_map>
#include <memory>
#include <algorithm>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <LinearMath/btConvexHullComputer.h>

// Bullet3 NarrowPhase / pipeline headers (adjust include paths as needed)
#include "Bullet3Collision/NarrowPhaseCollision/b3CpuNarrowPhase.h"
#include "Bullet3Common/b3Vector3.h"

inline btVector3 applyScaling(const btVector3& v, const btVector3& s) {
    return btVector3(v.x() * s.x(), v.y() * s.y(), v.z() * s.z());
}

inline void flattenVec3(const std::vector<btVector3>& v, std::vector<float>& out) {
	out.resize(v.size() * 3);
	for (size_t i = 0; i < v.size(); ++i) {
		out[i * 3 + 0] = (float)v[i].x();
		out[i * 3 + 1] = (float)v[i].y();
		out[i * 3 + 2] = (float)v[i].z();
	}
}

// Generate a simple UV-sphere point cloud for convex hull approximation
inline std::vector<btVector3> generateSpherePoints(float radius, int stacks = 12, int slices = 16) {
	std::vector<btVector3> pts;
	pts.reserve((size_t)stacks * (size_t)slices);
	for (int i = 0; i <= stacks; ++i) {
		float v = (float)i / (float)stacks;
		float phi = v * SIMD_PI; // 0..PI
		float y = std::cos(phi);
		float r = std::sin(phi);
		for (int j = 0; j < slices; ++j) {
			float u = (float)j / (float)slices;
			float theta = u * SIMD_2_PI; // 0..2PI
			float x = r * std::cos(theta);
			float z = r * std::sin(theta);
			pts.emplace_back(x * radius, y * radius, z * radius);
		}
	}
	return pts;
}

// Cylinder aligned along Y (Bullet’s default for btCylinderShape is usually Y axis)
inline std::vector<btVector3> generateCylinderPoints(float radius, float halfHeight, int slices = 24) {
	std::vector<btVector3> pts;
	pts.reserve((size_t)slices * 4);
	for (int i = 0; i < slices; ++i) {
		float a = (SIMD_2_PI * i) / slices;
		float x = radius * std::cos(a);
		float z = radius * std::sin(a);
		pts.emplace_back(x, +halfHeight, z);
		pts.emplace_back(x, -halfHeight, z);
		// Add inner ring offsets to improve hull quality
		float x2 = (radius * 0.7f) * std::cos(a + (SIMD_PI / slices));
		float z2 = (radius * 0.7f) * std::sin(a + (SIMD_PI / slices));
		pts.emplace_back(x2, +halfHeight, z2);
		pts.emplace_back(x2, -halfHeight, z2);
	}
	return pts;
}

// Box corners from half-extents
inline std::vector<btVector3> generateBoxCorners(const btVector3& halfExtents) {
	std::vector<btVector3> v;
	v.reserve(8);
	const float sx[2] = { -1.f, +1.f };
	for (int i = 0; i < 2; ++i)
	for (int j = 0; j < 2; ++j)
	for (int k = 0; k < 2; ++k) {
		v.emplace_back(
			sx[i] * halfExtents.x(),
			sx[j] * halfExtents.y(),
			sx[k] * halfExtents.z()
		);
	}
	return v;
}

// Extract vertices from btConvexHullShape (apply scaling)
inline std::vector<btVector3> extractConvexHullPoints(const btConvexHullShape* ch, const btVector3& scaling) {
	std::vector<btVector3> pts;
	int n = ch->getNumPoints();
#if (BT_BULLET_VERSION >= 285)
	const btVector3* unscaled = ch->getUnscaledPoints();
	pts.reserve(n);
	for (int i = 0; i < n; ++i) pts.emplace_back(applyScaling(unscaled[i], scaling));
#else
	pts.reserve(n);
	for (int i = 0; i < n; ++i) {
		btVector3 p;
		ch->getPoint(i, p);
		pts.emplace_back(applyScaling(p, scaling));
	}
#endif
	return pts;
}

// Extract mesh data (vertices + indices) from any btStridingMeshInterface
inline void extractFromStridingMeshInterface(const btStridingMeshInterface* smi,
									  const btVector3& scaling,
									  std::vector<float>& outVerts,    // xyz float array
									  std::vector<int>& outIndices) {
	const unsigned char* vertexbase = nullptr;
	const unsigned char* indexbase = nullptr;
	int numverts = 0;
	PHY_ScalarType type = PHY_INTEGER;
	int stride = 0;
	int numfaces = 0;
	PHY_ScalarType indicestype = PHY_INTEGER;
	int indexstride = 0;

	// Remove const_cast if calculateAabbBruteForce is const in your version
	btVector3 aabbMin, aabbMax;
	const_cast<btStridingMeshInterface*>(smi)->calculateAabbBruteForce(aabbMin, aabbMax); // optional

	btScalar scalingArr[3] = { scaling.x(), scaling.y(), scaling.z() };

	int numSubParts = smi->getNumSubParts();
	int baseVertex = 0;

	for (int part = 0; part < numSubParts; ++part) {
		smi->getLockedReadOnlyVertexIndexBase(
			&vertexbase, numverts, type, stride,
			&indexbase, indexstride, numfaces, indicestype, part
		);

		const bool is32 = (indicestype == PHY_INTEGER);
		const bool is16 = (indicestype == PHY_SHORT);

		size_t oldVertCount = outVerts.size() / 3;
		outVerts.reserve(outVerts.size() + (size_t)numverts * 3);
		outIndices.reserve(outIndices.size() + (size_t)numfaces * 3);

		for (int v = 0; v < numverts; ++v) {
			const float* pf = reinterpret_cast<const float*>(vertexbase + v * stride);
			btVector3 p(pf[0], pf[1], pf[2]);
			p = applyScaling(p, scaling);
			outVerts.push_back((float)p.x());
			outVerts.push_back((float)p.y());
			outVerts.push_back((float)p.z());
		}

		for (int f = 0; f < numfaces; ++f) {
			int i0, i1, i2;
			if (is32) {
				const int* idx = reinterpret_cast<const int*>(indexbase + f * indexstride);
				i0 = idx[0]; i1 = idx[1]; i2 = idx[2];
			} else if (is16) {
				const unsigned short* idx = reinterpret_cast<const unsigned short*>(indexbase + f * indexstride);
				i0 = idx[0]; i1 = idx[1]; i2 = idx[2];
			} else {
				// Unexpected format; skip
				continue;
			}
			// Rebase to global buffer
			outIndices.push_back((int)(oldVertCount + i0));
			outIndices.push_back((int)(oldVertCount + i1));
			outIndices.push_back((int)(oldVertCount + i2));
		}

		smi->unLockReadOnlyVertexBase(part);
	}
}

// Extract from btBvhTriangleMeshShape
inline void extractFromBvhTriangleMesh(const btBvhTriangleMeshShape* tm,
								std::vector<float>& outVerts,
								std::vector<int>& outIndices) {
	const btVector3 scaling = tm->getLocalScaling();
	const btStridingMeshInterface* smi = tm->getMeshInterface();
	extractFromStridingMeshInterface(smi, scaling, outVerts, outIndices);
}

// Extract from GImpact mesh
inline void extractFromGImpact(const btGImpactMeshShape* gm,
						std::vector<float>& outVerts,
						std::vector<int>& outIndices) {
	btGImpactMeshShape* g = const_cast<btGImpactMeshShape*>(gm);
	g->updateBound();
	btStridingMeshInterface* smi = g->getMeshInterface();
	const btVector3 scaling = g->getLocalScaling();
	extractFromStridingMeshInterface(smi, scaling, outVerts, outIndices);
}

// Build a convex hull from an arbitrary point cloud using btConvexHullComputer
inline std::vector<btVector3> computeConvexHull(const std::vector<btVector3>& points) {
	if (points.empty()) return {};
	std::vector<float> flat;
	flat.reserve(points.size() * 3);
	for (auto& p : points) { flat.push_back((float)p.x()); flat.push_back((float)p.y()); flat.push_back((float)p.z()); }

	btConvexHullComputer hull;
	hull.compute(&flat[0], sizeof(float) * 3, (int)points.size(), 0.0f, 0.0f);

	std::vector<btVector3> verts;
	verts.reserve(hull.vertices.size());
	for (int i = 0; i < hull.vertices.size(); ++i) {
		const btVector3& v = hull.vertices[i];
		verts.emplace_back(v);
	}
	return verts;
}

// Helper to register a convex hull with NarrowPhase
inline int registerConvex(b3GpuNarrowPhase* narrow,
				   const std::vector<btVector3>& verts) {
	if (!narrow || verts.size() < 4) return -1;
	std::vector<float> flat;
	flattenVec3(verts, flat);
	// Adjust the call to match your NarrowPhase signature if needed
	return narrow->registerConvexHullShape(
		flat.data(),               // vertex base (float*)
		sizeof(float) * 3,         // stride
		(int)verts.size(),         // num vertices
		b3MakeVector3(1.f, 1.f, 1.f) // scaling already baked in
	);
}

// Helper to register a concave mesh with NarrowPhase
/*inline int registerConcave(
    b3CpuNarrowPhase* narrow,
    const std::vector<float>& verts,
    const std::vector<int>& indices,
    const float* scaling = nullptr // Optional scaling with default// e.g., float scale[3] = {1.f, 1.f, 1.f};
) {
    if (!narrow || verts.empty() || indices.empty()) return -1;

    // Convert std::vector<float> (x,y,z,x,y,z,...) into b3AlignedObjectArray<b3Vector3>
    b3AlignedObjectArray<b3Vector3> bulletVerts;
    bulletVerts.resize(verts.size() / 3);
    for (size_t i = 0; i < bulletVerts.size(); ++i) {
        bulletVerts[i].setValue(
            verts[i * 3 + 0],
            verts[i * 3 + 1],
            verts[i * 3 + 2]
        );
    }

    // Convert std::vector<int> into b3AlignedObjectArray<int>
    b3AlignedObjectArray<int> bulletIndices;
    bulletIndices.resize(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        bulletIndices[i] = indices[i];
    }

    // Call the current API
    return narrow->registerConcaveMesh(&bulletVerts, &bulletIndices, scaling);
}*/

// Gather vertices from a compound’s children (transformed into parent space).
// For simplicity we gather convex samples and mesh vertices, then compute one global convex hull.
inline void gatherCompoundVertices(const btCompoundShape* comp, std::vector<btVector3>& out) {
	const int n = comp->getNumChildShapes();
	out.reserve(out.size() + (size_t)n * 64);
	for (int i = 0; i < n; ++i) {
		const btCollisionShape* cs = comp->getChildShape(i);
		const btTransform& t = comp->getChildTransform(i);
		const btVector3 scaling = cs->getLocalScaling();

		switch (cs->getShapeType()) {
			case BOX_SHAPE_PROXYTYPE: {
				const btBoxShape* box = static_cast<const btBoxShape*>(cs);
				auto verts = generateBoxCorners(box->getHalfExtentsWithMargin() * scaling);
				for (auto& v : verts) out.push_back(t * v);
			} break;
			case SPHERE_SHAPE_PROXYTYPE: {
				const btSphereShape* sph = static_cast<const btSphereShape*>(cs);
				auto verts = generateSpherePoints(sph->getRadius());
				for (auto& v : verts) out.push_back(t * applyScaling(v, scaling));
			} break;
			case CYLINDER_SHAPE_PROXYTYPE: {
				const btCylinderShape* cyl = static_cast<const btCylinderShape*>(cs);
				btVector3 he = cyl->getHalfExtentsWithMargin() * scaling;
				// Cylinder Y-axis: radius from XZ, halfHeight from Y
				float radius = std::max(he.x(), he.z());
				float hh = he.y();
				auto verts = generateCylinderPoints(radius, hh);
				for (auto& v : verts) out.push_back(t * v);
			} break;
			case CONVEX_HULL_SHAPE_PROXYTYPE: {
				const btConvexHullShape* ch = static_cast<const btConvexHullShape*>(cs);
				auto verts = extractConvexHullPoints(ch, scaling);
				for (auto& v : verts) out.push_back(t * v);
			} break;
			case TRIANGLE_MESH_SHAPE_PROXYTYPE: {
				// For meshes, we could extract triangles; for hull fallback, sample vertices only
				const btBvhTriangleMeshShape* tm = static_cast<const btBvhTriangleMeshShape*>(cs);
				std::vector<float> v; std::vector<int> idx;
				extractFromBvhTriangleMesh(tm, v, idx);
				for (size_t k = 0; k + 2 < v.size(); k += 3) {
					btVector3 p(v[k+0], v[k+1], v[k+2]);
					out.push_back(t * p);
				}
			} break;
			case GIMPACT_SHAPE_PROXYTYPE: {
				const btGImpactMeshShape* gm = static_cast<const btGImpactMeshShape*>(cs);
				std::vector<float> v; std::vector<int> idx;
				extractFromGImpact(gm, v, idx);
				for (size_t k = 0; k + 2 < v.size(); k += 3) {
					btVector3 p(v[k+0], v[k+1], v[k+2]);
					out.push_back(t * p);
				}
			} break;
			default: {
				// Best-effort: try convex approximation
				// No-op here; unsupported child types are skipped
			} break;
		}
	}
}

#endif // SHAPE_DATA_H