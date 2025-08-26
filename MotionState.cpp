#include "MotionState.h"

#include "BoneNode.h"
#include "RigidBody.h"

DefaultMotionState::DefaultMotionState(const glm::mat4& transform)
{
    // GLM → Bullet 변환 (열 우선 그대로)
    _transform.setFromOpenGLMatrix(glm::value_ptr(transform));
    _initTransform = _transform;
}

void DefaultMotionState::getWorldTransform(btTransform& worldTrans) const
{
    worldTrans = _transform;
}

void DefaultMotionState::setWorldTransform(const btTransform& worldTrans)
{
    _transform = worldTrans;
}

void DefaultMotionState::Reset()
{
    _transform = _initTransform;
}

void DefaultMotionState::ReflectGlobalTransform()
{

}

btTransform GlmToBt(const glm::mat4& m) {
    btTransform t; t.setFromOpenGLMatrix(glm::value_ptr(m)); return t;
}

glm::mat4 BtToGlm(const btTransform& t) {
    float m[16]; t.getOpenGLMatrix(m); return glm::make_mat4(m);
}

DynamicMotionState::DynamicMotionState(BoneNode* boneNode,
    const glm::mat4& offset,
    bool overrideBone)
    : _boneNode(boneNode)
    , _offset(offset)
    , _invOffset(glm::inverse(offset))
    , _override(overrideBone)
{
    assert(_boneNode && "DynamicMotionState requires a valid BoneNode*");
    Reset();
}

void DynamicMotionState::getWorldTransform(btTransform& worldTrans) const
{
    worldTrans = _transform;
}

void DynamicMotionState::setWorldTransform(const btTransform& worldTrans)
{
    _transform = worldTrans;
}

void DynamicMotionState::Reset()
{
    // Bullet이 원하는 초기 worldTransform = offset * bone_global
    const glm::mat4 global = _offset * _boneNode->_globalTransform;
    _transform = GlmToBt(global);
}

void DynamicMotionState::ReflectGlobalTransform()
{
    // Bullet 결과(world) -> bone_global = invOffset * world
    const glm::mat4 world = BtToGlm(_transform);
    const glm::mat4 result = _invOffset * world;

    if (_override) {
        _boneNode->_globalTransform = result;
        _boneNode->UpdateChildTransform();
    }
}

glm::mat4 GetMatrixFromBtTransform(const btTransform& t)
{
    float m[16];
    t.getOpenGLMatrix(m);            // Bullet은 col-major, OpenGL 행렬 포맷 그대로
    return glm::make_mat4(m);        // GLM도 col-major이므로 그대로 생성
}

// offset: (리짓바디 로컬) -> (본 글로벌) 변환
DynamicAndBoneMergeMotionState::DynamicAndBoneMergeMotionState(BoneNode* boneNode, const glm::mat4& offset, bool overrideBone)
    : _boneNode(boneNode)
    , _offset(offset)
    , _invOffset(glm::inverse(offset))
    , _override(overrideBone)
{
    Reset();
}

void DynamicAndBoneMergeMotionState::getWorldTransform(btTransform& worldTrans) const
{
    worldTrans = _transform;
}

void DynamicAndBoneMergeMotionState::setWorldTransform(const btTransform& worldTrans)
{
    _transform = worldTrans;
}

void DynamicAndBoneMergeMotionState::Reset()
{
    // 초기 world = offset * bone_global
    const glm::mat4 global = _offset * _boneNode->_globalTransform;
    _transform = GlmToBt(global);
}

void DynamicAndBoneMergeMotionState::ReflectGlobalTransform()
{
    // Bullet world 결과를 bone_global로 환산
    const glm::mat4 world = BtToGlm(_transform);
    glm::mat4 result = _invOffset * world;

    // 🔸 번역(translation)은 기존 본(global)의 것을 유지 (원 코드의 result.r[3] = global.r[3]에 해당)
    const glm::mat4 global = _boneNode->_globalTransform;
    result[3] = global[3];   // GLM은 col-major: [3]은 4번째 열(translation)

    if (_override) {
        _boneNode->_globalTransform = result;
        _boneNode->UpdateChildTransform();
    }
}

KinematicMotionState::KinematicMotionState(BoneNode* node, const glm::mat4& offset)
    : _boneNode(node), _offset(offset) {
}

void KinematicMotionState::getWorldTransform(btTransform& worldTrans) const
{
    glm::mat4 matrix;
    if (_boneNode)
    {
        // world = offset * bone_global
        matrix = _offset * _boneNode->_globalTransform;
    }
    else
    {
        matrix = _offset;
    }

    worldTrans.setFromOpenGLMatrix(glm::value_ptr(matrix));
}

void KinematicMotionState::setWorldTransform(const btTransform&)
{
    // Kinematic: Bullet이 결과를 쓰지 않음 (사용자 쪽이 구동)
}

void KinematicMotionState::Reset()
{
    // 필요 시 초기 상태 기억/복구 로직 추가 가능
}

void KinematicMotionState::ReflectGlobalTransform()
{
    // Kinematic은 본을 물리로부터 갱신하지 않음
}
