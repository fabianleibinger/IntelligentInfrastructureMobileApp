import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/blocs/userbloc.dart';
import 'package:parkingapp/dialogs/parkdialog.dart';
import 'package:parkingapp/dialogs/parkoutdialog.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/global.dart';
import 'package:wifi/wifi.dart';

final parkhausImage = 'assets/parkgarage-fasanengarten.jpg';
final parkhausImageHeight = 250;
final bottomMargin = 220;
List<String> _properties = [
  'Freie Parkplätze: 79',
  'Fahrzeugpräferenzen: keine'
];
bool _charge = false;

class MainPage extends StatefulWidget {
  final String apikey;

  const MainPage({Key key, this.apikey}) : super(key: key);
  @override
  _MainPageState createState() => _MainPageState();
}

class _MainPageState extends State<MainPage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
          title: Text('Vehicle', style: whiteHeader),
        ),
        drawer: Drawer(
          child: ListView(
            padding: EdgeInsets.zero,
            children: <Widget>[
              DrawerHeader(
                decoration: BoxDecoration(
                  color: green,
                ),
                child: Text('Parkhaus', style: blackHeader),
              ),
              ListTile(
                leading: Icon(Icons.message),
                title: Text('Messages'),
              ),
              ListTile(
                leading: Icon(Icons.account_circle),
                title: Text('Profile'),
              ),
              ListTile(
                leading: Icon(Icons.settings),
                title: Text('Settings'),
              ),
            ],
          ),
        ),
        floatingActionButton: FloatingActionButton.extended(
          onPressed: () {},
          label: Text('Fahrzeug einparken'),
        ),
        floatingActionButtonLocation: FloatingActionButtonLocation.centerFloat,
        body: Column(
          children: [
            Container(
              constraints: BoxConstraints(
                  maxHeight: MediaQuery.of(context).size.height - bottomMargin),
              child: Card(
                margin: EdgeInsets.all(0),
                elevation: 10,
                clipBehavior: Clip.antiAlias,
                child: Column(
                  children: [
                    ListTile(
                      //leading: Icon(Icons.location_on),
                      title: const Text('Parkgarage Fasanengarten'),
                      subtitle: Text(
                        'Tiefgarage',
                        style: TextStyle(color: Colors.black.withOpacity(0.6)),
                      ),
                    ),
                    Container(
                      height: parkhausImageHeight.toDouble(),
                      decoration: BoxDecoration(
                          image: DecorationImage(
                        image: AssetImage(parkhausImage),
                        fit: BoxFit.cover,
                      )),
                    ),
                    Flexible(
                      child: ListView(
                        shrinkWrap: true,
                        children: buildElements(_properties),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ],
        ));
  }

  List<Widget> buildElements(List<String> elements) {
    List<Widget> widgets = [];
    widgets.addAll(elements
        .map((element) => ListTile(
              title: Text(element),
            ))
        .toList());
    widgets.add(SwitchListTile(
      title: Text('Fahrzeug laden'),
      onChanged: (bool newValue) {
        setState(() {
          _charge = newValue;
        });
      },
      value: _charge,
    ));
    return widgets;
  }
}
