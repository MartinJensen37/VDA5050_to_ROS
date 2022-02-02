import json
import jsonschema
from jsonschema import validate

def get_schema():
    """This function loads the given schema available"""
    with open('../schemas/order.schema', 'r',  encoding='utf8') as file:
        schema = json.load(file)
        file.close()
    return schema


def validate_json(json_data):
    # Describe what kind of json you expect.
    execute_api_schema = get_schema()
    print(jsonschema.Draft3Validator(schema=execute_api_schema).is_valid(json_data))




with open('test.json', 'r',  encoding='utf8') as file:
    order = json.load(file)
    file.close()

# validate it
validate_json(order)
#print(msg)