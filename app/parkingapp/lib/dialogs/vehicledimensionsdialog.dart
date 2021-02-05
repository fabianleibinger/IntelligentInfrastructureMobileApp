import 'dart:ui';
import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/examplevehicle.dart';
import 'package:parkingapp/models/global.dart';
import 'constants.dart';
import 'dart:convert';

class VehicleDimensionsDialog extends StatefulWidget {
  VehicleDimensionsDialog({Key key}) : super(key: key);

  @override
  _VehicleDimensionsDialogState createState() =>
      _VehicleDimensionsDialogState();
}

class _VehicleDimensionsDialogState extends State<VehicleDimensionsDialog> {
  int selectedValue = 0;

  @override
  Widget build(BuildContext context) {
    return Dialog(
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(Constants.padding),
      ),
      elevation: 10,
      backgroundColor: white,
      child: contentBox(context),
    );
  }

  contentBox(context) {
    return Stack(
      children: <Widget>[
        Container(
          padding: EdgeInsets.all(20),
          height: 250,
          width: double.infinity,
          decoration: BoxDecoration(
            borderRadius: BorderRadius.all(Radius.circular(10)),
          ),
          child: Column(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              crossAxisAlignment: CrossAxisAlignment.center,
              children: <Widget>[
                FutureBuilder(
                    future: DefaultAssetBundle.of(context)
                        .loadString('assets/example-vehicles.json'),
                    builder: (context, snapshot) {
                      List<ExampleVehicle> exampleVehicles =
                          parseJson(snapshot.data.toString());

                      return exampleVehicles.isNotEmpty
                          ? Container(
                              height: 150,
                              width: double.infinity,
                              child: ListView.builder(
                                itemCount: exampleVehicles == null
                                    ? 0
                                    : exampleVehicles.length,
                                itemBuilder: (BuildContext context, int index) {
                                  return RadioListTile(
                                    value: index,
                                    groupValue: selectedValue,
                                    onChanged: (ind) {
                                      setState(() {
                                        selectedValue = ind;
                                        print(exampleVehicles[selectedValue]
                                            .name);
                                      });
                                    },
                                    title: Text(exampleVehicles[index].name),
                                  );
                                  /*Container(
                                    child: Row(
                                      children: <Widget>[
                                        Radio(
                                            value: exampleVehicles[index],
                                            groupValue: selectedValue,
                                            onChanged: (s) {
                                              setState(() {
                                                selectedValue = s;
                                              });
                                            }),
                                        Text(exampleVehicles[index].name)
                                      ],
                                    ),
                                  );*/
                                },
                              ))
                          : Center(
                              child: CircularProgressIndicator(),
                            );
                    }),
                FlatButton(
                    onPressed: () {
                      Navigator.pop(context);
                    },
                    child: Text("Close"))
              ]),
        ),
      ],
    );
  }

  List<ExampleVehicle> parseJson(String response) {
    if (response == null) {
      return [];
    }
    List<ExampleVehicle> list;
    /*final parsed = json.decode(response);
    list = List<ExampleVehicle>.from(
        parsed.map((model) => ExampleVehicle.fromJson(model)));*/

    list = (json.decode(response) as List)
        .map((e) => ExampleVehicle.fromJson(e))
        .toList();

    return list;
  }
}
