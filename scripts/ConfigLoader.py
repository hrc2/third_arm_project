#!/usr/bin/env python
import json
import importlib

# ROS import way
try:
    from scripts.toolbox import generateFile
# regular import way
except:
    from toolbox import generateFile

#JSON resource
#https://stackabuse.com/reading-and-writing-json-to-a-file-in-python/
#https://medium.com/python-pandemonium/json-the-python-way-91aac95d4041? maybe
#actually decent website: https://realpython.com/python-json/
#just kidding its actually this one
#source https://medium.com/python-pandemonium/json-the-python-way-91aac95d4041

def loadJSON(fileName):
    # fileName : str
    """loads the config file and returns the json data converted to the original object"""
    with open(fileName) as json_file:
        file = json.load(json_file, object_hook = decodeJSONFile)
    return file

def saveJSON(data, fileName, purpose, path = './'):
    # data, fileName : str, purpose : str
    """saves a new JSON file, returns fileName"""

    fileName = generateFile(path, fileName, '.json')

    with open(fileName, 'w+') as newJSONFile:
        json.dump(data, newJSONFile,default = encodeJSONFile, indent = 4)
    print('JSON file for', purpose, 'created')
    return fileName

def encodeJSONFile(data):
    """prepares the data from the config object to be saved as a new config file"""

    dataSet = {"__class__": data.__class__.__name__,"__module__": data.__module__}
    dataSet.update(data.__dict__)
    #for variable, value in vars(data).items():
    #    data.update(variable = value)
    return dataSet

def decodeJSONFile(data):
    """converts the data from the JSON file to a format readable by the target class"""
    if '__class__' in data:
        # Pop ensures we remove metadata from the dict to leave only the instance arguments
        class_name = data.pop("__class__")

        # Get the module name from the dict and import it
        module_name = data.pop("__module__")

        # We use the built in __import__ function since the module name is not yet known at runtime
        module = importlib.import_module(module_name)

        # Get the class from the module
        class_ = getattr(module,class_name)

        # Use dictionary unpacking to initialize the object
        info = class_(**data)
    else:
        info = data
    return info