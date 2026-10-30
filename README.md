# Camera gimbal

Shared stabilization and camera-axis articulation. The declarative mod is `0sfs/camera-gimbal`; the local package is `@0sfs/camera-gimbal`. The drone feature imports it lazily after applying an aircraft.

`createCameraGimbal` owns a fixed mounting base, yaw → roll → pitch joints, and a camera. `pose(body, pitchDeg, { rollDeg, yawDeg })` cancels combined body roll and pitch, aims at the requested pitch and heading offset, then rolls about the optical axis. Pitch is positive up; yaw is clockwise from aircraft heading; roll banks the image right. All overrides default to zero. The final body-relative orientation is decomposed into each physical joint, so pitching the camera does not pitch its mounting assembly. Joint locations remain authored asset data. A local sensor offset and vertical FOV keep sensor geometry in the aircraft profile.

`poseVisualGimbal(base, pitch, body, pitchDeg, overrides, { yaw, roll })` applies the same articulation to authored model pivots. Fixed mounting meshes remain outside the moving joints; yaw-stage meshes descend from yaw, roll-stage meshes from roll, and only the camera housing/lens descends from pitch. The component preserves the original two-node behavior when those optional stages are absent. `stabilizeGimbal` also preserves its four-argument API, with optional roll/yaw overrides as a fifth argument.

`slewGimbal` bounds a target and its angular speed. A cockpit camera adopted by the rig returns to its original parent, position, and rotation on view change or disposal.

This component knows nothing about WebGPU capture, JSBSim, brands, remote controls, or panorama sequences. Capture stays in the flight feature. Gimbal stabilization is separate from the Aircraft/Momentum/Planet chase-camera frame selection. This local component has not been published.

Topics: `0sfs-mod` and `0sfs-component-mod`. [Modding guide](https://github.com/0SFS/docs/blob/main/modding.md) describes discovery and installation.
