import { afterEach, describe, expect, it } from 'vitest';
import { Axis, NullEngine, Quaternion, Scene, TransformNode, Vector3 } from '@babylonjs/core';
import { createCameraGimbal, poseVisualGimbal, stabilizeGimbal } from '../src/index';
const DEG = Math.PI / 180;
const engines: NullEngine[] = [];
function scene() {
    const engine = new NullEngine(); engines.push(engine);
    const scene = new Scene(engine); scene.useRightHandedSystem = true;
    return scene;
}
afterEach(() => { for (const engine of engines.splice(0)) engine.dispose(); });
function expectRotation(actual: Quaternion, expected: Quaternion) {
    expect(Math.abs(Quaternion.Dot(actual.normalizeToNew(), expected.normalizeToNew()))).toBeCloseTo(1, 6);
}
function rotationOf(node: TransformNode): Quaternion {
    return Quaternion.FromRotationMatrix(node.computeWorldMatrix(true).getRotationMatrix());
}
function frame(scene: Scene, roll: number, pitch: number, heading: number) {
    const body = new TransformNode('airframe', scene);
    body.rotationQuaternion = Quaternion.RotationYawPitchRoll(Math.PI - heading, -pitch, roll);
    return body;
}
describe('articulated camera axes', () => {
    it('moves only the pitch joint for manual pitch on a level aircraft', () => {
        const s = scene(), rig = createCameraGimbal(s, { name: 'camera', offset: { x: 0, y: 0, z: 0 }, vfovRad: 1 });
        for (const pitch of [-90, -45, 20]) {
            rig.pose({ rollRad: 0, pitchRad: 0 }, pitch);
            expectRotation(rig.base.rotationQuaternion!, Quaternion.Identity());
            expectRotation(rig.yaw.rotationQuaternion!, Quaternion.Identity());
            expectRotation(rig.roll.rotationQuaternion!, Quaternion.Identity());
            expectRotation(rig.pitch.rotationQuaternion!, Quaternion.RotationAxis(Axis.X, -pitch * DEG));
        }
        rig.dispose();
    });
    it('cancels arbitrary body roll and pitch without moving the fixed mounting base', () => {
        const s = scene();
        for (const [roll, pitch, heading] of [[.4, -.3, 1.2], [-.8, .7, -.5], [1.4, -.9, 2.7], [0, 0, 0]]) {
            const body = frame(s, roll, pitch, heading);
            const rig = createCameraGimbal(s, { name: 'camera', offset: { x: .1, y: -.2, z: .3 }, vfovRad: 1 });
            rig.base.parent = body;
            rig.pose({ rollRad: roll, pitchRad: pitch }, -35);
            expectRotation(rig.base.rotationQuaternion!, Quaternion.Identity());
            expect(rig.base.position.asArray()).toEqual([.1, -.2, .3]);
            expectRotation(rotationOf(rig.pitch), Quaternion.RotationYawPitchRoll(Math.PI - heading, 35 * DEG, 0));
            rig.dispose(); body.dispose();
        }
    });
    it('aims yaw clockwise from heading and rolls around the optical axis at steep pitch', () => {
        const s = scene(), body = frame(s, .6, -.4, 1.1);
        const rig = createCameraGimbal(s, { name: 'camera', offset: { x: 0, y: 0, z: 0 }, vfovRad: 1 });
        rig.base.parent = body;
        for (const [pitch, yaw, roll] of [[0, 20, 0], [-90, 25, 15], [20, -30, -10], [-45, 50, 40]]) {
            rig.pose({ rollRad: .6, pitchRad: -.4 }, pitch, { yawDeg: yaw, rollDeg: roll });
            const expected = Quaternion.RotationYawPitchRoll(Math.PI - 1.1 - yaw * DEG, -pitch * DEG, roll * DEG);
            expectRotation(rotationOf(rig.pitch), expected);
            const world = rig.camera.getViewMatrix(true).clone().invert();
            const forward = Vector3.TransformNormal(new Vector3(0, 0, -1), world).normalize();
            expect(forward.x).toBeCloseTo(Math.sin(1.1 + yaw * DEG) * Math.cos(pitch * DEG), 5);
            expect(forward.y).toBeCloseTo(Math.sin(pitch * DEG), 5);
            expect(forward.z).toBeCloseTo(-Math.cos(1.1 + yaw * DEG) * Math.cos(pitch * DEG), 5);
        }
        body.rotationQuaternion = Quaternion.RotationYawPitchRoll(Math.PI, 0, 0);
        rig.pose({ rollRad: 0, pitchRad: 0 }, 0, { rollDeg: 20 });
        rig.pitch.computeWorldMatrix(true);
        const world = rig.camera.getViewMatrix(true).clone().invert();
        expect(Vector3.TransformNormal(Vector3.Right(), world).normalize().y).toBeCloseTo(-Math.sin(20 * DEG));
        rig.dispose();
    });
    it('keeps visible stage rotations identical to the sensor rig, including roll singularities', () => {
        const s = scene(), body = frame(s, .3, .2, 2);
        const rig = createCameraGimbal(s, { name: 'sensor', offset: { x: 0, y: 0, z: 0 }, vfovRad: 1 });
        rig.base.parent = body;
        const base = new TransformNode('visual-base', s), yaw = new TransformNode('visual-yaw', s);
        const roll = new TransformNode('visual-roll', s), pitch = new TransformNode('visual-pitch', s);
        base.parent = body; yaw.parent = base; roll.parent = yaw; pitch.parent = roll;
        for (const [bodyRoll, bodyPitch, targetPitch, targetYaw, targetRoll] of [
            [.3, .2, -70, 15, 10], [0, 0, 0, 35, 90], [0, 0, 0, -35, -90], [1.7, .8, 20, -20, 35],
        ]) {
            const pose = { rollRad: bodyRoll, pitchRad: bodyPitch };
            body.rotationQuaternion = Quaternion.RotationYawPitchRoll(Math.PI - 2, -bodyPitch, bodyRoll);
            const overrides = { rollDeg: targetRoll, yawDeg: targetYaw };
            rig.pose(pose, targetPitch, overrides);
            poseVisualGimbal(base, pitch, pose, targetPitch, overrides, { yaw, roll });
            expectRotation(rotationOf(pitch), rotationOf(rig.pitch));
            const expected = Quaternion.RotationYawPitchRoll(Math.PI - 2 - targetYaw * DEG, -targetPitch * DEG, targetRoll * DEG);
            expectRotation(rotationOf(pitch), expected);
            for (const node of [base, yaw, roll, pitch]) expect(node.rotationQuaternion!.asArray().every(Number.isFinite)).toBe(true);
        }
        rig.dispose();
    });
    it('retains the legacy two-node API and gives it the same sensor orientation with overrides', () => {
        const s = scene(), body = frame(s, -.7, .4, -.5);
        const base = new TransformNode('legacy-base', s), pitch = new TransformNode('legacy-pitch', s);
        base.parent = body; pitch.parent = base;
        const pose = { rollRad: -.7, pitchRad: .4 }, overrides = { rollDeg: -10, yawDeg: 15 };
        poseVisualGimbal(base, pitch, pose, -25, overrides);
        const expected = Quaternion.RotationYawPitchRoll(Math.PI + .5 - 15 * DEG, 25 * DEG, -10 * DEG);
        expectRotation(rotationOf(pitch), expected);
        const qBase = Quaternion.Identity(), qPitch = Quaternion.Identity();
        stabilizeGimbal(qBase, qPitch, pose, -25 * DEG, overrides);
        expectRotation(qBase, base.rotationQuaternion!); expectRotation(qPitch, pitch.rotationQuaternion!);
    });
});
