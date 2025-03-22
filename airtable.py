import requests
import json 
import keyboard
import time

''' Format: 'https://api.airtable.com/v0/BaseID/tableName '''
URL = 'https://api.airtable.com/v0/app4OyYDgJbKOh4b8/Table%201'

Headers = {
    'Authorization':'Bearer pat6qoPknqxSh1GoQ.11c5682d5a60918a4402f88aa6c96048a10ed695d8043871393bb6b9bfc9bab8',
    'Content-Type': 'application/json'
}

try:
    linear_new = [0, 0, 0]
    angular_new = [0, 0, 0]
    while True:
        r = requests.get(url = URL, headers = Headers, params = {})
        data = r.json()

        linear = [data["records"][1]["fields"]["X"], data["records"][1]["fields"]["Y"], data["records"][1]["fields"]["Z"]]
        angular = [data["records"][0]["fields"]["X"], data["records"][0]["fields"]["Y"], data["records"][0]["fields"]["Z"]]


        # new values to add

        if linear == linear_new and angular == angular_new:
            pass
        else:
            # Prepare data for updating
            new_data = {
                "records": [
                    {
                        "id": data["records"][0]["id"],
                        "fields": {
                            "X": angular_new[0],
                            "Y": angular_new[1],
                            "Z": angular_new[2],
                        },
                    }, {
                        "id": data["records"][1]["id"],
                        "fields": {
                            "X": linear_new[0],
                            "Y": linear_new[1],
                            "Z": linear_new[2],
                        },
                    }
                ]
            }

            new_data =  json.dumps(new_data)

            #post request 
            r = requests.patch(URL, data=new_data, headers=Headers)
            print(r.status_code) #200 if it works
        time.sleep(.001)
        if keyboard.is_pressed('w'):
            linear_new[0] = 1
        if keyboard.is_pressed('s') and not keyboard.is_pressed('w'):
            linear_new[0] = -0.6
        if not keyboard.is_pressed('w') and not keyboard.is_pressed('s'):
            linear_new[0] = 0

        if keyboard.is_pressed('a'):
            angular_new[2] = 0.7
        if keyboard.is_pressed('d') and not keyboard.is_pressed('a'):
            angular_new[2] = -0.7
        if not keyboard.is_pressed('d') and not keyboard.is_pressed('a'):
            angular_new[2] = 0
except KeyboardInterrupt:
    print("done")
