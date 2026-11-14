"""Scene assembly is separate from controllers and episode orchestration."""
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco


def build_scene(config):
    if config.robot == "g1":
        asset_dir = Path(config.asset_dir).resolve()
        source = asset_dir / "scene_g1.xml"
        if not source.is_file():
            raise FileNotFoundError("G1 assets missing. Run: g1-carry fetch-assets")
        root = ET.parse(source).getroot()
        root.find("compiler").set("meshdir", str(asset_dir / "assets"))
        for side in ("left", "right"):
            body = root.find(f".//body[@name='{side}_wrist_yaw_link']")
            ET.SubElement(body, "site", name=f"robot_{side}", pos="0.08 0 0", size="0.015", rgba="0 1 0 1")
    else:
        root = ET.fromstring('''<mujoco model="surrogate_force_bench">
          <worldbody><light pos="0 -2 4"/><geom name="floor" type="plane" size="0 0 .05" rgba=".2 .25 .3 1"/>
          <body name="pelvis" pos="0 0 .7"><joint name="x" type="slide" axis="1 0 0" damping="5"/>
          <joint name="y" type="slide" axis="0 1 0" damping="5"/>
          <geom type="box" size=".15 .2 .2" mass="20" rgba=".25 .6 .85 1"/>
          <site name="robot_left" pos=".2 .18 0"/><site name="robot_right" pos=".2 -.18 0"/>
          </body></worldbody><actuator><motor joint="x" ctrlrange="-100 100"/>
          <motor joint="y" ctrlrange="-100 100"/></actuator></mujoco>''')
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", str(config.timestep))
    option.set("integrator", "implicitfast")
    option.set("gravity", "0 0 -9.81")
    world = root.find("worldbody")
    # Human avatar is visual only. Its finite-strength hand impedance supplies force.
    human = ET.SubElement(world, "body", name="human_avatar", mocap="true", pos="1.5 0 .7")
    for attributes in [dict(type="capsule", fromto="0 0 -.1 0 0 .4", size=".12"),
                       dict(type="sphere", pos="0 0 .6", size=".1"),
                       dict(type="capsule", fromto="0 .08 -.1 0 .12 -.65", size=".055"),
                       dict(type="capsule", fromto="0 -.08 -.1 0 -.12 -.65", size=".055")]:
        ET.SubElement(human, "geom", **attributes, contype="0", conaffinity="0", rgba=".95 .55 .2 1")
    for side, y in (("left", .18), ("right", -.18)):
        ET.SubElement(human, "geom", type="capsule", fromto=f"0 {y} .3 -.25 {y} 0", size=".04", contype="0", conaffinity="0", rgba=".95 .55 .2 1")
    if config.carry:
        payload = ET.SubElement(world, "body", name="payload", pos=".7 0 .7")
        ET.SubElement(payload, "freejoint", name="payload_free")
        ET.SubElement(payload, "geom", name="payload_box", type="box",
            size=f"{config.payload.length/2} .22 {config.payload.thickness/2}",
            mass=str(config.payload.mass), rgba=".65 .38 .17 1")
        for end, x in (("robot", -config.payload.length / 2), ("human", config.payload.length / 2)):
            for side, y in (("left", .18), ("right", -.18)):
                ET.SubElement(payload, "site", name=f"payload_{end}_{side}", pos=f"{x} {y} 0", size=".015", rgba="0 1 0 1")
                if config.payload.handle_length:
                    outside = x + (-1 if end == "robot" else 1) * config.payload.handle_length
                    ET.SubElement(payload, "geom", name=f"handle_{end}_{side}", type="capsule",
                        fromto=f"{x} {y} 0 {outside} {y} 0", size=".012", mass="0",
                        contype="0", conaffinity="0", rgba=".5 .5 .5 1")
        contact = root.find("contact")
        if contact is None:
            contact = ET.SubElement(root, "contact")
        if config.robot == "g1":
            for side in ("left", "right"):
                ET.SubElement(contact, "exclude", body1="payload", body2=f"{side}_wrist_yaw_link")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
