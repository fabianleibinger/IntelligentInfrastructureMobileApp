import 'package:flutter/material.dart';
import 'package:parkingapp/models/global.dart';

//basic building blocks for dialogs
class Constants {

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
        createBackTextButton(context, red, cancelButtonText),
        createTextButton(
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
        createBackTextButton(context, red, cancelButtonText),
        createTextButton(
            context, green, confirmButtonText, confirmButtonNextPage)
      ],
    );
  }

  //returns an alert dialog with text and one button
  static getAlertDialogOneButtonNoTitle(BuildContext context, String content,
      String confirmButtonText, String nextPage) {
    return AlertDialog(
      content: Text(content),
      actions: [createTextButton(context, red, confirmButtonText, nextPage)],
    );
  }

  //returns an alert dialog with text and one back button
  static getAlertDialogOneBackButtonNoTitle(
      BuildContext context, String content, String confirmButtonText) {
    return AlertDialog(
      content: Text(content),
      actions: [createBackTextButton(context, green, confirmButtonText)],
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
        createBackTextButton(context, green, confirmButtonText),
      ],
    );
  }

  //creates a button that closes a dialog
  static TextButton createBackTextButton(BuildContext context, Color color, String text) {
    return TextButton(
      style: TextButton.styleFrom(primary: color),
      onPressed: () => Navigator.pop(context),
      child: Text(text),
    );
  }

  //creates a button. [nextPage] defines the page to be called by button
  static TextButton createTextButton(
      BuildContext context, Color color, String text, String nextPage) {
    return TextButton(
      style: TextButton.styleFrom(primary: color),
      onPressed: () => Navigator.pushReplacementNamed(context, nextPage),
      child: Text(text),
    );
  }
}
