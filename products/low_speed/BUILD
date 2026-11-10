load("@core//runtime:product.bzl", "runtime_product")

package(default_visibility = ["//visibility:public"])

COMMON_PRODUCT_FILES = {
    "//products/low_speed/conf:global_flagfile": "conf/global_flagfile.txt",
    "//products/low_speed/conf:vehicle_config_data": "conf/vehicles/default/vehicle_param.pb.txt",
    "@wheelos_common//modules/common/vehicle_model:vehicle_model_config_data": "conf/vehicles/default/vehicle_model_config.pb.txt",
}

runtime_product(
    name = "devkit_x86",
    files = COMMON_PRODUCT_FILES | {
        "//products/low_speed/launch:devkit_x86.launch": "launch/low_speed/devkit_x86.launch",
    },
    modules = [
        "//tools/hardware:hardware_module",
        "@wheelos_control//modules/control:module",
        "@wheelos_localization//modules/rtk_localization:module",
        "@wheelos_planning//modules/open_space_planning:module",
        "@wheelos_transform//modules/transform:module",
    ],
    package_file_name = "devkit_x86.tar.gz",
)

runtime_product(
    name = "devkit_x86_ndt",
    files = COMMON_PRODUCT_FILES | {
        "//products/low_speed/launch:devkit_x86_ndt.launch": "launch/low_speed/devkit_x86_ndt.launch",
    },
    modules = [
        "//tools/hardware:hardware_module",
        "@wheelos_control//modules/control:module",
        "@wheelos_localization//modules/ndt_localization:module",
        "@wheelos_planning//modules/planning:module",
        "@wheelos_transform//modules/transform:module",
    ],
    package_file_name = "devkit_x86_ndt.tar.gz",
)

alias(
    name = "runtime",
    actual = ":devkit_x86",
)
