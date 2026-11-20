from glob import glob

from setuptools import find_packages, setup

setup(
    name="alohamini_duel",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/alohamini_duel"]),
        ("share/alohamini_duel", ["package.xml"]),
        (
            "share/alohamini_duel/config",
            glob("config/*.yaml") + glob("config/*.pgm") + glob("config/*.xml"),
        ),
        ("share/alohamini_duel/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="anncatto",
    maintainer_email="anncatto@users.noreply.github.com",
    description="AlohaMini EDU 双车无接触底盘联调",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "duel_base = alohamini_duel.base_node:main",
            "shared_duel_sim = alohamini_duel.sim_node:main",
            "duel_referee = alohamini_duel.referee_node:main",
            "duel_check = alohamini_duel.preflight:main",
            "duel_stop_watch = alohamini_duel.stop_watch:main",
            "duel_route = alohamini_duel.route:main",
        ]
    },
)
