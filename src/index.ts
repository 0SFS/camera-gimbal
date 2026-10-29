import { Axis, Matrix, Quaternion, TransformNode, UniversalCamera, Vector3, type Scene } from '@babylonjs/core';
export interface GimbalPose {
    rollRad: number;
    pitchRad: number;
}
/** Camera offsets from the level frame at aircraft heading, in degrees. */
export interface GimbalOverrides {
    /** Positive banks the image to the right about the optical axis. */
    rollDeg?: number;
    /** Positive looks clockwise from the aircraft heading. */
    yawDeg?: number;
}
export interface GimbalStages {
    yaw: TransformNode;
    roll: TransformNode;
}
export interface CameraGimbalOptions {
    name: string;
    offset: {
        x: number;
        y: number;
        z: number;
    };
    vfovRad: number;
    minZ?: number;
    maxZ?: number;
    cockpitCameraName?: string;
}
const DEG = Math.PI / 180;
const correction = Quaternion.Identity(), desired = Quaternion.Identity(), local = Quaternion.Identity();
const rotation = Matrix.Identity();

/** The desired sensor orientation relative to the aircraft; its heading cancels algebraically. */
function cameraRotationToRef(body: GimbalPose, pitchRad: number, overrides: GimbalOverrides, result: Quaternion): void {
    Quaternion.RotationAxisToRef(Axis.Z, -body.rollRad, correction);
    Quaternion.RotationAxisToRef(Axis.X, body.pitchRad, desired);
    correction.multiplyInPlace(desired);
    Quaternion.RotationYawPitchRollToRef(-(overrides.yawDeg ?? 0) * DEG, -pitchRad, (overrides.rollDeg ?? 0) * DEG, desired);
    correction.multiplyToRef(desired, result);
}
/**
 * Legacy two-node rig: cancel body roll/pitch on the base, then aim the sensor.
 * Positive pitch is up; roll and yaw are optional stabilized-frame offsets.
 */
export function stabilizeGimbal(base: Quaternion, pitch: Quaternion, body: GimbalPose, pitchRad: number, overrides: GimbalOverrides = {}): void {
    Quaternion.RotationAxisToRef(Axis.Z, -body.rollRad, correction);
    Quaternion.RotationAxisToRef(Axis.X, body.pitchRad, desired);
    correction.multiplyToRef(desired, base);
    Quaternion.RotationYawPitchRollToRef(-(overrides.yawDeg ?? 0) * DEG, -pitchRad, (overrides.rollDeg ?? 0) * DEG, pitch);
}
export function slewGimbal(current: number, target: number, dt: number, min: number, max: number, rate: number): number {
    const desired = Math.max(min, Math.min(max, target)), step = Math.max(0, dt) * rate;
    return current + Math.max(-step, Math.min(step, desired - current));
}
/**
 * An articulated rig keeps its mounting base fixed and puts each rotation on
 * its mechanical yaw(Y) → roll(Z) → pitch(X) joint. With no explicit stages,
 * preserve the original two-node API for older visual assets.
 */
export function poseVisualGimbal(base: TransformNode, pitch: TransformNode, body: GimbalPose, pitchDeg: number,
    overrides: GimbalOverrides = {}, stages?: GimbalStages): void {
    base.rotationQuaternion ??= Quaternion.Identity();
    pitch.rotationQuaternion ??= Quaternion.Identity();
    if (!stages) {
        stabilizeGimbal(base.rotationQuaternion, pitch.rotationQuaternion, body, pitchDeg * DEG, overrides);
        return;
    }
    stages.yaw.rotationQuaternion ??= Quaternion.Identity();
    stages.roll.rotationQuaternion ??= Quaternion.Identity();
    cameraRotationToRef(body, pitchDeg * DEG, overrides, local);
    local.toRotationMatrix(rotation);
    // R = Ry(yaw) Rz(roll) Rx(pitch). Babylon stores column axes in m[0..2], etc.
    // Keep roll in [-90, 90] and use the remaining joints to represent the same
    // sensor pose. At the roll singularity choose yaw=0, keeping a finite pose.
    const m = rotation.m;
    const roll = Math.asin(Math.max(-1, Math.min(1, m[1])));
    const ordinary = Math.hypot(m[0], m[2]) > 1e-6;
    const yaw = ordinary ? Math.atan2(-m[2], m[0]) : 0;
    const tilt = ordinary ? Math.atan2(-m[9], m[5]) : Math.atan2(m[6], m[10]);
    base.rotationQuaternion.set(0, 0, 0, 1);
    Quaternion.RotationAxisToRef(Axis.Y, yaw, stages.yaw.rotationQuaternion);
    Quaternion.RotationAxisToRef(Axis.Z, roll, stages.roll.rotationQuaternion);
    Quaternion.RotationAxisToRef(Axis.X, tilt, pitch.rotationQuaternion);
}
export function createCameraGimbal(scene: Scene, options: CameraGimbalOptions) {
    const base = new TransformNode(options.name + '-base', scene);
    base.position.copyFromFloats(options.offset.x, options.offset.y, options.offset.z);
    base.rotationQuaternion = Quaternion.Identity();
    const yaw = new TransformNode(options.name + '-yaw', scene);
    yaw.parent = base;
    yaw.rotationQuaternion = Quaternion.Identity();
    const roll = new TransformNode(options.name + '-roll', scene);
    roll.parent = yaw;
    roll.rotationQuaternion = Quaternion.Identity();
    const pitch = new TransformNode(options.name + '-pitch', scene);
    pitch.parent = roll;
    pitch.rotationQuaternion = Quaternion.Identity();
    const stages = { yaw, roll };
    const camera = new UniversalCamera(options.name + '-camera', Vector3.Zero(), scene);
    camera.parent = pitch;
    camera.rotationQuaternion = Quaternion.RotationYawPitchRoll(Math.PI, 0, 0);
    camera.fov = options.vfovRad;
    camera.fovMode = UniversalCamera.FOVMODE_VERTICAL_FIXED;
    camera.minZ = options.minZ ?? .02;
    camera.maxZ = options.maxZ ?? 250000;
    let adopted: UniversalCamera | null = null;
    let home: {
        parent: UniversalCamera['parent'];
        position: Vector3;
        rotation: Quaternion | null;
    } | null = null;
    const restore = () => {
        if (adopted && home) {
            adopted.parent = home.parent;
            adopted.position.copyFrom(home.position);
            adopted.rotationQuaternion = home.rotation?.clone() ?? null;
        }
        adopted = null;
        home = null;
    };
    return {
        base, yaw, roll, pitch, camera,
        pose(body: GimbalPose, pitchDeg: number, overrides: GimbalOverrides = {}) {
            poseVisualGimbal(base, pitch, body, pitchDeg, overrides, stages);
        },
        followView(active: UniversalCamera | null) {
            const cockpit = scene.getCameraByName(options.cockpitCameraName ?? 'cockpit-camera') as UniversalCamera | null;
            if (!cockpit || active !== cockpit) {
                restore();
                return;
            }
            if (adopted !== cockpit) {
                restore();
                home = { parent: cockpit.parent, position: cockpit.position.clone(), rotation: cockpit.rotationQuaternion?.clone() ?? null };
                adopted = cockpit;
                cockpit.parent = pitch;
                cockpit.position.setAll(0);
            }
            cockpit.rotationQuaternion = Quaternion.RotationYawPitchRoll(Math.PI, 0, 0);
        },
        dispose() {
            restore();
            camera.dispose();
            pitch.dispose();
            roll.dispose();
            yaw.dispose();
            base.dispose();
        },
    };
}
