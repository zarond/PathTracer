#include "mikktspace.h"
#include "compute_tangents.h"

namespace {
    using namespace app;

    int getNumFaces(const SMikkTSpaceContext* context) {
        Mesh* mesh = static_cast<Mesh*>(context->m_pUserData);
        return mesh->indices.size() / 3;
    }

    int getNumVerticesOfFace(const SMikkTSpaceContext* context, const int faceIdx) {
        return 3; // always triangles
    }

    void getPosition(const SMikkTSpaceContext* context, float posOut[], const int faceIdx, const int vertIdx) {
        Mesh* mesh = static_cast<Mesh*>(context->m_pUserData);
        auto index = mesh->indices[faceIdx * 3 + vertIdx];
        memcpy(posOut, &mesh->vertices[index].position, sizeof(fvec3));
    }

    void getNormal(const SMikkTSpaceContext* context, float normOut[], const int faceIdx, const int vertIdx) {
        Mesh* mesh = static_cast<Mesh*>(context->m_pUserData);
        auto index = mesh->indices[faceIdx * 3 + vertIdx];
        memcpy(normOut, &mesh->vertices[index].normal, sizeof(fvec3));
    }

    void getTexCoord(const SMikkTSpaceContext* context, float uvOut[], const int faceIdx, const int vertIdx) {
        Mesh* mesh = static_cast<Mesh*>(context->m_pUserData);
        auto index = mesh->indices[faceIdx * 3 + vertIdx];
        memcpy(uvOut, &mesh->vertices[index].uv, sizeof(fvec2));
    }

    void setTSpaceBasic(const SMikkTSpaceContext* context, const float tangent[], const float sign,
        const int faceIdx, const int vertIdx) {
        Mesh* mesh = static_cast<Mesh*>(context->m_pUserData);
        auto index = mesh->indices[faceIdx * 3 + vertIdx];
        memcpy(&mesh->vertices[index].tangent, tangent, sizeof(fvec3));
        mesh->vertices[index].tangent[3] = sign;
    }
}

namespace app {

    TangentSpaceHelper::TangentSpaceHelper() {
        iface_.m_getNumFaces = getNumFaces;
        iface_.m_getNumVerticesOfFace = getNumVerticesOfFace;
        iface_.m_getPosition = getPosition;
        iface_.m_getNormal = getNormal;
        iface_.m_getTexCoord = getTexCoord;
        iface_.m_setTSpaceBasic = setTSpaceBasic;

        context_.m_pInterface = &iface_;
    }

    void TangentSpaceHelper::compute_tangents(Mesh& mesh) {
        context_.m_pUserData = &mesh;
        bool success = genTangSpaceDefault(&context_);
        assert(success);
    }
}