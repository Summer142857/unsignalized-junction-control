from xml.etree.ElementTree import ElementTree,Element

def read_xml(path):
    tree = ElementTree()
    tree.parse(path)
    return tree


def write_xml(tree, path):
    tree.write(path, encoding="utf-8", xml_declaration=True)


def alterDemand(path, probability):
    tree = read_xml(path)
    root = tree.getroot()
    for flow_node in root.findall('flow'):
        flow_node.set("probability", str(probability))
    write_xml(tree, path)
    print("Alter demand factor to " + str(probability))