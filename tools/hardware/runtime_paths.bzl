"""Stable deployment paths, independent of hardware source module layout."""

_SOURCE_ROOTS = {
    Label("@hardware.audio//:component").repo_name: "modules/drivers/microphone",
    Label("@hardware.camera//:component").repo_name: "modules/drivers/camera",
    Label("@hardware.camera_gst//:component").repo_name: "modules/drivers/camera_gst",
    Label("@hardware.canbus//:tool").repo_name: "modules/drivers/canbus",
    Label("@hardware.gnss//:component").repo_name: "modules/drivers/gnss",
    Label("@hardware.hal//:CPPLINT.cfg").repo_name: "modules/drivers/hal",
    Label("@hardware.image_decompress//:component").repo_name: "modules/drivers/tools/image_decompress",
    Label("@hardware.lidar//:component").repo_name: "modules/drivers/lidar",
    Label("@hardware.radar//:conti").repo_name: "modules/drivers/radar",
    Label("@hardware.vehicle//:component").repo_name: "modules/canbus",
    Label("@hardware.video//:component").repo_name: "modules/drivers/video",
}

def deployment_path(file):
    path = file.short_path
    if path.startswith("../"):
        path = path.split("/", 2)[2]
    root = _SOURCE_ROOTS.get(file.owner.repo_name)
    if root == None:
        return path
    if root == "modules/canbus" and path.startswith("brands/"):
        path = "vehicle/" + path.removeprefix("brands/")
    return root + "/" + path
