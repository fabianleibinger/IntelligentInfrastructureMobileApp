import 'dart:ui';
import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:parkingapp/models/global.dart';
import 'constants.dart';

class ParkPreferencesDialog extends StatefulWidget {
  ParkPreferencesDialog({Key key}) : super(key: key);

  @override
  _ParkPreferencesDialogState createState() => _ParkPreferencesDialogState();
}

class _ParkPreferencesDialogState extends State<ParkPreferencesDialog> {
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
              children: <Widget>[]),
        ),
      ],
    );
  }
}
