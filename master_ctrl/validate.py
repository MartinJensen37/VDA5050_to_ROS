import json
import jsonschema
from jsonschema import validate

def get_schema(schema):
    """This function loads the given schema available"""
    with open('../schemas/'+schema, 'r',  encoding='utf8') as file:
        schema = json.load(file)
        file.close()
    return schema


def validate_json(json_data, schema):
    # Describe what kind of json you expect.
    execute_api_schema = get_schema(schema)
    print(schema, "is valid? ", jsonschema.Draft3Validator(schema=execute_api_schema).is_valid(json_data))


schema_list=['connection.schema', 'order.schema', 'instantActions.schema', 'state.schema', 'visualization.schema']
msg_list = ['connection_msg.json', 'order_msg.json', 'instant_action_msg.json', 'state_msg.json', 'visualization_msg.json']

for msg, schema in zip(msg_list, schema_list):
    with open('../messages/'+msg, 'r',  encoding='utf8') as file:
        msg_data = json.load(file)
    file.close()
    validate_json(msg_data, schema)
