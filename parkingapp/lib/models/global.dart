import 'package:flutter/material.dart';

Color black = Colors.black;
Color blue = new Color(0xFF0E2356);
Color green = new Color(0xFF007749);
Color grey = new Color(0xFF807F84);
Color lightgreen = new Color(0xFF2FAE7A);
Color red = new Color(0xFFBF1E00);
Color white = Colors.white;

TextStyle blackText = new TextStyle(
    fontFamily: 'Avenir',
    fontWeight: FontWeight.bold,
    color: black,
    fontSize: 30);

TextStyle lightGreyText = new TextStyle(
    fontFamily: 'Avenir',
    fontWeight: FontWeight.bold,
    color: grey,
    fontSize: 30);

TextStyle smallBlackText =
    new TextStyle(fontFamily: 'Avenir', color: black, fontSize: 20);

TextStyle smallLightGreyText =
    new TextStyle(fontFamily: 'Avenir', color: grey, fontSize: 20);

TextStyle tinyLightGreyText =
    new TextStyle(fontFamily: 'Avenir', color: grey, fontSize: 10);

TextStyle blackDialogHeader = new TextStyle(
    fontFamily: 'Avenir',
    fontWeight: FontWeight.bold,
    color: black,
    fontSize: 30);

TextStyle whiteHeader =
    new TextStyle(fontFamily: 'Avenir', color: white, fontSize: 20);

TextStyle blackHeader = new TextStyle(
    fontFamily: 'Avenir',
    fontWeight: FontWeight.bold,
    color: black,
    fontSize: 30);

TextStyle whiteButtonText = new TextStyle(
    fontFamily: 'Avenir',
    fontWeight: FontWeight.bold,
    color: white,
    fontSize: 30);

TextStyle lightGreyButtonText = new TextStyle(
    fontFamily: 'Avenir',
    fontWeight: FontWeight.bold,
    color: grey,
    fontSize: 30);

TextStyle greenButtonText = new TextStyle(
    fontFamily: 'Avenir',
    fontWeight: FontWeight.bold,
    color: green,
    fontSize: 30);

TextStyle redButtonText = new TextStyle(
    fontFamily: 'Avenir',
    fontWeight: FontWeight.bold,
    color: red,
    fontSize: 30);

// App theme
ThemeData themeData = ThemeData(
  primaryColor: green,
  accentColor: lightgreen,
  visualDensity: VisualDensity.adaptivePlatformDensity,
  dialogBackgroundColor: Colors.transparent,
);
