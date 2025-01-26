import requests
import json

releases_s = requests.get("https://api.github.com/repos/whooprobotics/reveillib/releases?per_page=100").content
releases = json.loads(releases_s)

depot = []

for release in releases:
    if(len(release["assets"]) > 0):
        depot_entry = {}
        depot_entry["metadata"] = {}
        depot_entry["metadata"]["location"] = release["assets"][0]["browser_download_url"]
        depot_entry["name"] = "reveillib"
        depot_entry["py/object"] = "pros.conductor.templates.base_template.BaseTemplate"
        depot_entry["target"] = "v5"
        depot_entry["version"] = release["tag_name"]
        depot_entry["supported_kernels"] = "3.8.0"

        depot.append(depot_entry)

print(json.dumps(depot, indent=2))
