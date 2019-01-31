import json, io
import re
import xml.etree.ElementTree as ET
import argparse


def parse_pose(pose):
    
    p_list = pose.split(' ')

    return (float(p_list[0]), float(p_list[1]), float(p_list[2]), float(p_list[3]), float(p_list[4]), float(p_list[5]))

def get_id_text_from_name(name, regex):

    ids = re.compile(regex)
    match = ids.search(name)
    if match:
	    return match.group(1)
    else:
	    return '0'


if __name__=='__main__' :

	
    parser = argparse.ArgumentParser()
    parser.add_argument("input_file", help="Path to the .world file to process")
    parser.add_argument("object_name", help="String that the name of the object starts with, e.g.: 'aruco_4x4'")
    parser.add_argument("regex", help="Regex rule string to match and extract the marker value ID from the name of the object, e.g.: '4x4_(\d\d?)'")
    args = parser.parse_args()

    #_filename = 'test_track.world'
    _filename = args.input_file
    _output_filename = 'output.json'
    #_regex = r"4x4_(\d\d?)"
    _regex = args.regex
    #_object_name = 'aruco_4x4'
    _object_name = args.object_name

    
    xmldoc = ET.parse(_filename)
    world = xmldoc.find('world').find('state')
    
    _id = 0 # unique id per gate starting at 0
    output = list()

    for model in world.findall('model'):
        model_name = model.attrib['name']
        if model_name.startswith( _object_name):
            model_object = {}
            model_object['id'] = _id
            _id+=1
            model_object['name'] = model_name                
            #print model_name
            model_object['marker_value'] = get_id_text_from_name(model_name,_regex)
            pose_text = model.find('pose').text
            model_object['pose'] = parse_pose(pose_text)
           
            print  model_object['id'], model_name, model_object['marker_value'], model_object['pose']
            output.append(model_object)
                        
    if len(output) > 0:
	    #print json.dumps(output, sort_keys = True, indent = 4, ensure_ascii = False)
	    with io.open(_output_filename, 'w', encoding="utf-8") as outfile:
		    outfile.write(unicode(json.dumps(output, sort_keys = True, indent = 4, ensure_ascii = False)))
		    outfile.close()
		    print '* Saved output to', _output_filename



