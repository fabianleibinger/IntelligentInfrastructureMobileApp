import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:parkingapp/bloc/blocs/userbloc.dart';
import 'package:parkingapp/dialogs/parkdialog.dart';
import 'package:parkingapp/dialogs/parkoutdialog.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/global.dart';
import 'package:wifi/wifi.dart';

class MainPage extends StatefulWidget {
  final String apikey;

  const MainPage({Key key, this.apikey}) : super(key: key);
  @override
  _MainPageState createState() => _MainPageState();
}

class _MainPageState extends State<MainPage> {
  int anzParkplaetze = 79;
  bool laden = false;
  bool vehicleChargable = true;
  List<Vehicle> vehicles;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Vehicle', style: whiteHeader),
        backgroundColor: green,
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
      body: Container(
        child: Column(
          children: <Widget>[
            Column(
              children: <Widget>[
                Card(
                  elevation: 10,
                  child: Column(
                    children: <Widget>[
                      ListTile(
                        title: Text("Parkgarage Fasanengarten",
                            style: blackHeader),
                        subtitle: Text("Tiefgarage", style: tinyLightGreyText),
                        leading: Icon(
                          Icons.location_on,
                          color: black,
                          size: 40,
                        ),
                      ),
                      /*Container(
                        width: double.infinity,
                        child: FittedBox(
                          child: Image.asset(
                            'assets/images/parkgarage_am_fasanengarten.jpg',
                          ),
                        ),
                      ),*/
                      Container(
                        width: double.infinity,
                        height: 250,
                        child: ClipRRect(
                          child: FittedBox(
                            fit: BoxFit.fitWidth,
                            alignment: Alignment.bottomCenter,
                            child: ConstrainedBox(
                              constraints: BoxConstraints(
                                  minWidth: 1, minHeight: 1), // here
                              child: Image.asset(
                                'assets/images/parkgarage_am_fasanengarten.jpg',
                              ),
                            ),
                          ),
                        ),
                      ),
                      ListTile(
                        title: Text(
                            "Freie Parkplätze: " + anzParkplaetze.toString(),
                            style: blackText),
                      ),
                      ListTile(
                        title: Text("Fahrzeugpräferenzen: ", style: blackText),
                      ),
                      getChargeVehicle(context)
                    ],
                  ),
                )
              ],
            ),
            Padding(
                padding: const EdgeInsets.all(30),
                child: ElevatedButton(
                    onPressed: () {
                      vehicleChargable
                          ? vehicleChargable = false
                          : vehicleChargable = true;
                      refresh();
                    },
                    child: Text("FAHRZEUG EINPARKEN", style: whiteButtonText),
                    style: ButtonStyle(
                        backgroundColor:
                            MaterialStateProperty.all<Color>(green),
                        shape: MaterialStateProperty.all<OutlinedBorder>(
                            RoundedRectangleBorder(
                                borderRadius: BorderRadius.circular(5.0))),
                        elevation: MaterialStateProperty.all<double>(10.0))))
          ],
        ),
      ),
    );
  }

  void refresh() {
    setState(() {
      build(context);
    });
  }

  Widget getChargeVehicle(BuildContext context) {
    return vehicleChargable
        ? SwitchListTile(
            title: Text("Fahrzeug laden", style: blackText),
            value: laden,
            onChanged: (bool value) {
              setState(() {
                laden = value;
              });
            })
        : SwitchListTile(
            title: Text("Fahrzeug laden", style: lightGreyText),
            value: false,
            onChanged: null);
  }
}
