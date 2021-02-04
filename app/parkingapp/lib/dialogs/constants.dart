import 'package:flutter/material.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';

//basic building blocks for dialogs

class Constants {
  Constants._();

  static const double padding = 20;
  static const double avatarRadius = 45;

  static createParkAlertDialog(
      BuildContext context,
      String title,
      String content,
      String cancelButtonText,
      String confirmButtonText,
      String confirmButtonNextPage) {
    return showDialog(
        context: context,
        builder: (context) {
          return AlertDialog(
            backgroundColor: white,
            title: Text(title),
            content: Text(content),
            actions: [
              FlatButton(
                textColor: red,
                onPressed: () {
                  Navigator.pop(context);
                },
                child: Text(cancelButtonText),
              ),
              FlatButton(
                textColor: green,
                onPressed: () {
                  Navigator.pushReplacementNamed(context, Routes.park);
                },
                child: Text(confirmButtonText),
              ),
            ],
          );
        });
  }
}
