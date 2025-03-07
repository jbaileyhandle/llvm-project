// REQUIRES: amdgpu-registered-target

// RUN: %clang -### --target=x86_64-linux-gnu \
// RUN:   --offload-arch=gfx803 --offload-arch=gfx900 \
// RUN:   --cuda-device-only -nogpuinc -nogpulib -c \
// RUN:   %s 2>&1 | FileCheck %s

// CHECK-NOT: error:

// CHECK: [[CLANG:".*clang.*"]] "-cc1" "-triple" "amdgcn-amd-amdhsa"
// CHECK-SAME: "-fcuda-is-device"
// CHECK-SAME: "-target-cpu" "gfx803"
// CHECK-SAME: {{.*}} "-o" [[OBJ_DEV_A_803:".*o"]] "-x" "hip"

// CHECK: [[LLD: ".*lld.*"]] "-flavor" "gnu" "-m" "elf64_amdgpu" "--no-undefined" "-shared"
// CHECK-SAME: "-o" "[[IMG_DEV_A_803:.*out]]" [[OBJ_DEV_A_803]]

// CHECK: [[CLANG:".*clang.*"]] "-cc1" "-triple" "amdgcn-amd-amdhsa"
// CHECK-SAME: "-emit-obj"
// CHECK-SAME: "-fcuda-is-device"
// CHECK-SAME: "-target-cpu" "gfx900"
// CHECK-SAME: {{.*}} "-o" [[OBJ_DEV_A_900:".*o"]] "-x" "hip"

// CHECK: [[LLD]] "-flavor" "gnu" "-m" "elf64_amdgpu" "--no-undefined" "-shared"
// CHECK-SAME: "-o" "[[IMG_DEV_A_900:.*out]]" [[OBJ_DEV_A_900]]

// CHECK: [[BUNDLER:".*clang-offload-bundler"]] "-type=o"
// CHECK-SAME: "-targets={{.*}},hip{{.*}}-amdgcn-amd-amdhsa--gfx803,hip{{.*}}-amdgcn-amd-amdhsa--gfx900"
// CHECK-SAME: "-input={{.*}}" "-input=[[IMG_DEV_A_803]]" "-input=[[IMG_DEV_A_900]]" "-output=[[BUNDLE_A:.*hipfb]]"
