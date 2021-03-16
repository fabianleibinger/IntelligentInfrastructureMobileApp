import 'package:flutter/material.dart';
import 'package:parkingapp/models/global.dart';

//basic building blocks for dialogs
class Constants {
  //TODO remove
  static const double padding = 20;
  static const double avatarRadius = 45;

  //returns an alert dialog with title, text and two buttons
  static getAlertDialog(
      BuildContext context,
      String title,
      String content,
      String cancelButtonText,
      String confirmButtonText,
      String confirmButtonNextPage) {
    return AlertDialog(
      title: Text(title),
      content: Text(content),
      actions: [
        createBackFlatButton(context, red, cancelButtonText),
        createFlatButton(
            context, green, confirmButtonText, confirmButtonNextPage)
      ],
    );
  }

  //returns an alert dialog with text and two buttons
  static getAlertDialogNoTitle(
      BuildContext context,
      String content,
      String cancelButtonText,
      String confirmButtonText,
      String confirmButtonNextPage) {
    return AlertDialog(
      content: Text(content),
      actions: [
        createBackFlatButton(context, red, cancelButtonText),
        createFlatButton(
            context, green, confirmButtonText, confirmButtonNextPage)
      ],
    );
  }

  //returns an alert dialog with text and one button
  static getAlertDialogOneButtonNoTitle(BuildContext context, String content,
      String confirmButtonText, String nextPage) {
    return AlertDialog(
      content: Text(content),
      actions: [createFlatButton(context, red, confirmButtonText, nextPage)],
    );
  }

  //returns an alert dialog with text and one back button
  static getAlertDialogOneBackButtonNoTitle(
      BuildContext context, String content, String confirmButtonText) {
    return AlertDialog(
      content: Text(content),
      actions: [createBackFlatButton(context, green, confirmButtonText)],
    );
  }

  //returns a confirmation dialog with title and one button,
  //[tiles] Widget defines the content.
  //this is the basic dialog template for this app
  static getConfirmationDialog(BuildContext context, String title,
      String confirmButtonText, Widget tiles) {
    return AlertDialog(
      title: Text(title),
      content: Column(
        mainAxisSize: MainAxisSize.min,
        children: [tiles],
      ),
      actions: [
        createBackFlatButton(context, green, confirmButtonText),
      ],
    );
  }

  //creates a button that closes a dialog
  static createBackFlatButton(BuildContext context, Color color, String text) {
    return FlatButton(
      textColor: color,
      onPressed: () => Navigator.pop(context),
      child: Text(text),
    );
  }

  //creates a button. [nextPage] defines the page to be called by button
  static createFlatButton(BuildContext context, Color color, String text,
      String nextPage) {
    return FlatButton(
      textColor: color,
      onPressed: () => Navigator.pushReplacementNamed(context, nextPage),
      child: Text(text),
    );
  }
}
