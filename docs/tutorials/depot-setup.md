# Setting up the ReveilLib depot

To set up your ReveilLib Depot, you will first need to locate your conductor.pros file. This is a json file containing your pros conductor configuration.

Once you've found this, run it through [Unminify](https://unminify.com) to make it readable.

Locate the `depots` key in the JSON file and then add the following depot:

```json
"revup": {
        "config": {},
        "config-schema": {},
        "location": "https://whooprobotics.github.io/ReveilLib/revup.json",
        "name": "revup",
        "py/object": "pros.conductor.depots.http_depot.HttpDepot",
        "remote_templates": [],
        "last_remote_update": { "py/object": "datetime.datetime", "__reduce__": [{ "py/type": "datetime.datetime" }, ["B+cIGAwRHg4fRA=="]] },
        "update_frequency": { "py/reduce": [{ "py/type": "datetime.timedelta" }, { "py/tuple": [0, 60, 0] }] }

}
```

Once this is done, save the file and run 

```sh
pros c query --force-refresh
```

This will set up the RevUp depot, allowing you to easily download and install ReveilLib.

---
**NOTE**

In recent versions of PROS-cli, `pros c query` has been removed. Instead, you should instead use

```sh
pros c query-templates --force-refresh
```

---

To use ReveilLib from the depot, just go into your project directory and run

```sh
pros c apply reveillib
```

To update ReveilLib, refresh your template index and upgrade ReveilLib

```sh
pros c query-templates --force-refresh
pros c upgrade reveillib
```