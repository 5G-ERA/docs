# Graph relations

This documents represents the model for the representations of the relations on the Graph. 
Each object consists of the Property describing the initiating object, the Relation name, relation attributes and end object. 
Both initiating object and end object have properties that allow to recognize the type of an object presented on the graph and its properties. 
Relation name property contains the type of the relation between objects and relation attributes property contains the list of the key-value paris that describe the relation in greater detail.
```json
{
"Relations":
    [
        {   
            "InitiatesFrom": {
                "Id": "guid",
                "Type": "Edge",
                "Name": "Office Luton"
            },
            "RelationName": "CanReach",
            "RelationAttributes": [],
            "PointsTo": {
                "Id": "guid",
                "Type": "Cloud",
                "Name": "AWS Ireland"
            }
        },
        {
            "RelationName": "Provides",
            "RelationAttributes": [
                {"WORKING_FOR": "ROBOT_2"}
            ],
            "PointsTo": {
                "Id": "guid",
                "Type": "Action",
                "Name": "Object recognition"
            }
        },
        {
            "RelationName": "CanReach",
            "RelationAttributes": [],
            "PointsTo": {
                "Id": "guid",
                "Type": "Cloud",
                "Name": "AWS London"
            }
        },
    ]
}

```