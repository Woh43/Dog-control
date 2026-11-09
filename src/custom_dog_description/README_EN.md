# Custom Dog description

[简体中文](README.md) | **English**

This self-contained ROS 2 package includes three URDFs, 17 materialized binary STL meshes, joint/battery configuration, and a display launch file. It requires no adjacent model checkout or RL runtime.

The model weighs **15.15 kg**: 13.84916 kg upstream CAD mass plus a 1.30084 kg battery at the `base` origin. The battery is already merged into the 7.85084 kg base mass, with updated center of mass and parallel-axis inertia. Its intrinsic inertia remains a point-mass approximation because its dimensions are unknown.

`custom_dog.urdf` is canonical for dynamics and hardware. The Gazebo point-foot variant removes only eight thigh/calf collision elements; the selective-collision variant retains another upstream collision proxy set. All three share inertial, visual, and joint data. The canonical SHA-256 is `bf01f817cfb7fe3d0112333b7c6e767c441a80c03bd78f73a82c5c3b36540741`, unchanged from the tested November 3 model. Importing resources does not establish a new physics or hardware-motion acceptance.

Source: [Woh43/custom_dog_stack](https://github.com/Woh43/custom_dog_stack), commit `6c776de8deb76b8fd565cd80f16ac6a4cc8b635d`, package `ros2/src/custom_dog_description`. That commit still contains the unloaded model; the battery correction was applied locally before this import. Files preserve the current tested model workspace's bytes. [model_manifest.json](config/model_manifest.json) records upstream provenance, per-file size/SHA-256, LFS content identifiers, and the battery adjustment. No policies, training code, or deployment programs are included.

The upstream declarations differ: its root [LICENSE.upstream](LICENSE.upstream) is MIT, copyright 2026 h43, while its description `package.xml` declares `Proprietary` and maintainer hsc. Both declarations are preserved; this import does not resolve or replace them. Build/test integration and dependency metadata are maintained locally.

Display without accessing motor ports:

```bash
ros2 launch custom_dog_description display.launch.py
```

Run the offline resource check without ROS or a simulator:

```bash
python3 src/custom_dog_description/test/check_model_resources.py src/custom_dog_description
```

With `BUILD_TESTING` enabled, this check is also registered as CTest `check_model_resources`. It validates file/LFS integrity, mesh references, mass, physical inertia, battery composition, joint ordering, and collision-variant consistency. Resource changes require updated provenance and validation appropriate to their physical effect.
