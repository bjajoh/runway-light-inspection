import requests
from datetime import datetime
import csv

#Connection Variables
databaseURL = "https://www.ilocator.com/api/v2/users/token"
syncURL = url = "https://www.ilocator.com/api/v2/objects/sync/1"
databaseAuthUser = "DEMO";
#This is bad practice.
databaseAuthPass = "PW"
#And setting up the list of fields for the CSV
objectFieldNames = ['id', 'latitude', 'longitude', 'part_nmbr', 'serial_nmbr', 'information', 'information2', 'dimension', 'weight', 'origin', 'mobileapp_version_number', 'gps_precision', 'gps_age', 'gps_altitude', 'organizations_id', 'users_id', 'created', 'modified', 'modified_merged', 'crud', 'deleted', 'modified_timezone', 'object_categories_id', 'object_types_id', 'object_events_id', 'object_statuses_id']


def getAPIKey:
    #Initialize request values
    payload = {
    'username':databaseAuthUser,
    'password': databaseAuthPass}
    files = []
    #For the headers of the request, this can be hardcoded for the moment since there's
    #(at the moment at least) no other possibilities.
    tokenHeaders = {
      'Accept': 'application/json'
    }
    try:
        #Send off to ilocator, get response and encode as a JSON object.
        tokenAqResponse = requests.request("POST", databaseURL, headers=tokenHeaders, data = payload, files = files)
        #Conversion of raw binary response data to something usable.
        tokenResponseJSON = tokenAqResponse.json()
        print(tokenResponseJSON["success"])

        #Data will be converted to a python dictionary, then accessed properly.
        tokenResponseData = tokenResponseJSON["data"]
        if tokenResponseJSON["success"] :# == "True":
            print("Successfully obtained API key, default valid for 24 hours. API key written to token.txt\n\n")

        APItoken = tokenResponseData["token"]

        #Write API token to text file
        APIfile = open("token.txt","w")
        APIfile.write(APItoken)
        APIfile.close()
        return 1
    except:
        return 0

def getDBObjects:

    #Initialize data sync request values
    syncPayload = {'synchronized_last': ''} ###This will need to be updated with the accurate timings, but that can be fully included later
    syncHeaders = {
        'Accept' : 'application/json',
        'Application-Authorization' : 'Bearer '+APItoken
    }
    syncFiles = []
    try:
        dataPostResp = requests.request("POST", syncURL, headers = syncHeaders, data = syncPayload, files = syncFiles)

        #print(dataPostResp.text.encode('utf8'))
        dataResponseJSON = dataPostResp.json()
        objectList = dataResponseJSON["data"]
        objectList = objectList["objects"]

        #Now that we've got the objects from the database downloaded and extracted, put them in a CSV.
        #Note: This does every object, not just the lights. TODO: Figure out the specific object_categories_id or object_types_id that matches the lights that we're concerned with.
        with open('objects.csv', 'wb') as csvFile:
            writer = csv.DictWriter(csvFile, fieldnames = objectFieldNames)
            writer.writeheader()
            for tempObj in objectList:
                writer.writerow(tempObj)
        return 1
    except:
        return 0
def syncToDB:
    #Pending, need to fix a CSV to dictionary import issue with line handling.
    return 0
