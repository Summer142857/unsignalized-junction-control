import json

def read_config():
    filename = "../ConflictConfig.txt"
    file = open(filename, 'r')
    js = file.read()
    dic = json.loads(js)
    file.close()
    return dic