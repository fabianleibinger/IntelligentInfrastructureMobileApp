import 'dart:ui';
import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:parkingapp/models/global.dart';
import 'constants.dart';

// example for a popup dialog triggered by a button or something

class ExampleDialog extends StatefulWidget {
  final String apikey;
  ExampleDialog({Key key, this.apikey}) : super(key: key);

  @override
  _ExampleDialogState createState() => _ExampleDialogState();
}

class _ExampleDialogState extends State<ExampleDialog> {
  @override
  Widget build(BuildContext context) {
    return Dialog(
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(Constants.padding),
      ),
      elevation: 0,
      backgroundColor: Colors.transparent,
      child: contentBox(context),
    );
  }

  contentBox(context) {
    TextEditingController title = new TextEditingController();
    TextEditingController deadline = new TextEditingController();
    return Stack(
      children: <Widget>[
        Container(
          padding: EdgeInsets.all(20),
          height: 300,
          width: double.infinity,
          decoration: BoxDecoration(
            borderRadius: BorderRadius.all(Radius.circular(10)),
          ),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            crossAxisAlignment: CrossAxisAlignment.center,
            children: <Widget>[
              Text("Add New Task"),
              Container(
                child: TextField(
                  controller: title,
                  decoration: InputDecoration(
                      hintText: "Name of task",
                      enabledBorder: UnderlineInputBorder(
                          borderSide: BorderSide(color: Colors.white))),
                ),
              ),
              Container(
                child: TextField(
                  controller: deadline,
                  decoration: InputDecoration(
                      hintText: "Deadline",
                      enabledBorder: UnderlineInputBorder(
                          borderSide: BorderSide(color: Colors.white))),
                ),
              ),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                crossAxisAlignment: CrossAxisAlignment.center,
                children: <Widget>[
                  RaisedButton(
                      color: Colors.red.withOpacity(0.75),
                      child: Text(
                        "Cancel",
                      ),
                      onPressed: () {
                        Navigator.pop(context);
                      }),
                  RaisedButton(
                      color: Colors.red.withOpacity(0.75),
                      child: Text(
                        "Add",
                      ),
                      onPressed: () {
                        if (title.text != "" && deadline.text != "") {
                          // this should happen if you press the button
                        }
                      }),
                ],
              )
            ],
          ),
        ),
      ],
    );
  }
}
