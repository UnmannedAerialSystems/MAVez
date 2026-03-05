# THIS FILE CONVERTS PRE EXISTING MAVLINK DIALECTS INTO PYTHON ENUMS. THE GENERATED ENUMS ARE NOT ORIGINAL

import xml.etree.ElementTree as ET
import argparse


def parse_mavlink_messages(xml_path: str) -> dict[str, int]:
    tree = ET.parse(xml_path)
    root = tree.getroot()

    messages_el = root if root.tag == "messages" else root.find(".//messages")
    if messages_el is None:
        raise ValueError("No <messages> section found in XML.")

    result = {}
    for msg in messages_el.findall("message"):
        name = msg.get("name")
        msg_id = msg.get("id")
        if name is not None and msg_id is not None:
            result[name] = int(msg_id)
    return result


def write_enum_file(messages: dict[str, int], output_path: str) -> None:
    lines = [
        "from enum import Enum",
        "",
        "",
        "class MavMessageType(Enum):",
        '    """',
        "    Enum for MAVLink message types.",
        '    """',
        "",
    ]
    for name, id_ in sorted(messages.items(), key=lambda x: x[1]):
        lines.append(f"    {name} = {id_}")

    with open(output_path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("xml_path", help="Path to MAVLink XML file")
    parser.add_argument("output_path", help="Path for generated Python enum file")
    args = parser.parse_args()

    messages = parse_mavlink_messages(args.xml_path)
    write_enum_file(messages, args.output_path)
    print(f"Wrote {len(messages)} message types to {args.output_path}")