import 'package:flutter/material.dart';
import 'package:parkingapp/models/global.dart';

/// Basic building blocks for dialogs.
class Constants {
  static const double padding = 20;
  static const double avatarRadius = 45;

  /// Returns an alert dialog with [title], text [content], one back button and one confirm button.
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
        getBackTextButton(context, red, cancelButtonText),
        getTextButton(context, green, confirmButtonText, confirmButtonNextPage)
      ],
    );
  }

  /// Returns an alert dialog with text [content], one back button and one confirm button.
  static getAlertDialogNoTitle(
      BuildContext context,
      String content,
      String cancelButtonText,
      String confirmButtonText,
      String confirmButtonNextPage) {
    return AlertDialog(
      content: Text(content),
      actions: [
        getBackTextButton(context, red, cancelButtonText),
        getTextButton(context, green, confirmButtonText, confirmButtonNextPage)
      ],
    );
  }

  /// Returns an alert dialog with text [content] and one confirm button.
  static getAlertDialogOneButtonNoTitle(BuildContext context, String content,
      String confirmButtonText, String nextPage) {
    return AlertDialog(
      content: Text(content),
      actions: [getTextButton(context, red, confirmButtonText, nextPage)],
    );
  }

  /// Returns an alert dialog with text [content] and one back button.
  static getAlertDialogOneBackButtonNoTitle(
      BuildContext context, String content, String confirmButtonText) {
    return AlertDialog(
      content: Text(content),
      actions: [getBackTextButton(context, green, confirmButtonText)],
    );
  }

  /// Returns a confirmation dialog with [title], widget content [tiles] and one button.
  static getConfirmationDialog(
      BuildContext context, String title, String buttonText, Widget tiles) {
    return AlertDialog(
      title: Text(title),
      content: Column(
        mainAxisSize: MainAxisSize.min,
        children: [tiles],
      ),
      actions: [
        getBackTextButton(context, green, buttonText),
      ],
    );
  }

  /// Returns a button that closes a dialog, shows [text] and has a [color].
  static TextButton getBackTextButton(
      BuildContext context, Color color, String text) {
    return TextButton(
      style: TextButton.styleFrom(primary: color),
      onPressed: () => Navigator.pop(context),
      child: Text(text),
    );
  }

  /// Returns a button that opens a page [nextPage], shows [text] and has a [color].
  static TextButton getTextButton(
      BuildContext context, Color color, String text, String nextPage) {
    return TextButton(
      style: TextButton.styleFrom(primary: color),
      onPressed: () => Navigator.pushNamedAndRemoveUntil(
          context, nextPage, (route) => false),
      child: Text(text),
    );
  }
}
