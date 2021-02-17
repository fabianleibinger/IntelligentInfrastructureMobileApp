import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/global.dart';
import 'package:wifi/wifi.dart';
// example for a page (mainpage)

class ParkPage extends StatefulWidget {
  static const String routeName = '/parkpage';

  const ParkPage({Key key}) : super(key: key);
  @override
  _ParkPageState createState() => _ParkPageState();
}

class _ParkPageState extends State<ParkPage> {
  @override
  Widget build(BuildContext context) {}
}
