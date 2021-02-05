import 'package:flutter/material.dart';
import 'package:parkingapp/models/global.dart';

//basic building blocks for dialogs

class Constants {
  Constants._();

  static const double padding = 20;
  static const double avatarRadius = 45;

  static createAlertDialog(
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
              createBackFlatButton(context, red, cancelButtonText),
              createFlatButton(
                  context, green, confirmButtonText, confirmButtonNextPage)
            ],
          );
        });
  }

  static createAlertDialogNoTitle(
      BuildContext context,
      String content,
      String cancelButtonText,
      String confirmButtonText,
      String confirmButtonNextPage) {
    return showDialog(
        context: context,
        builder: (context) {
          return AlertDialog(
            backgroundColor: white,
            content: Text(content),
            actions: [
              createBackFlatButton(context, red, cancelButtonText),
              createFlatButton(
                  context, green, confirmButtonText, confirmButtonNextPage)
            ],
          );
        });
  }

  static createAlertDialogOneBackButtonNoTitle(
      BuildContext context, String content, String confirmButtonText) {
    return showDialog(
        context: context,
        builder: (context) {
          return AlertDialog(
            backgroundColor: white,
            content: Text(content),
            actions: [createBackFlatButton(context, green, confirmButtonText)],
          );
        });
  }

  static createConfirmationDialog(BuildContext context, String title,
      String confirmButtonText, Widget tiles) {
    return showDialog(
        context: context,
        builder: (context) {
          return AlertDialog(
            title: Text(title),
            content: Column(
              mainAxisSize: MainAxisSize.min,
              children: [],
            ),
            actions: [
              createBackFlatButton(context, green, confirmButtonText),
            ],
          );
        });
  }

  static createBackFlatButton(BuildContext context, Color color, String text) {
    return FlatButton(
      textColor: color,
      onPressed: () => Navigator.pop(context),
      child: Text(text),
    );
  }

  static createFlatButton(
      BuildContext context, Color color, String text, String nextPage) {
    return FlatButton(
      textColor: color,
      onPressed: () => Navigator.pushReplacementNamed(context, nextPage),
      child: Text(text),
    );
  }
}
