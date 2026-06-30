from glob import glob
from setuptools import setup

package_name = "k1_qr_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*")),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
        ("share/" + package_name + "/models/k1_glider", glob("models/k1_glider/*")),
        ("share/" + package_name + "/models/k1_glider_cam", glob("models/k1_glider_cam/*")),
        ("share/" + package_name + "/models/k1_glider_arm", glob("models/k1_glider_arm/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="boosterk1",
    maintainer_email="jangarabliss@gmail.com",
    description="SLAM + Nav2 navigate-to-QR for the Booster K1, with a lightweight 2-D test sim.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mini_sim = k1_qr_nav.mini_sim:main",
            "qr_goal = k1_qr_nav.qr_goal:main",
            "gz_qr_truth = k1_qr_nav.gz_qr_truth:main",
            "drive_to_qr = k1_qr_nav.drive_to_qr:main",
            "odom_tf = k1_qr_nav.odom_tf:main",
            "arm_reach = k1_qr_nav.arm_reach:main",
            "mobile_scan = k1_qr_nav.mobile_scan:main",
            "arm_scan = k1_qr_nav.arm_scan:main",
            "booster_bridge = k1_qr_nav.booster_bridge:main",
            "barcode_locate = k1_qr_nav.barcode_locate:main",
            "cart_locate = k1_qr_nav.cart_locate:main",
            "cart_approach = k1_qr_nav.cart_approach:main",
            "cart_track = k1_qr_nav.cart_track:main",
            "head_search = k1_qr_nav.head_search:main",
        ],
    },
)
