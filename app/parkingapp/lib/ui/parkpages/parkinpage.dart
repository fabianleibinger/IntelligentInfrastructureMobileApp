import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:image_size_getter/file_input.dart';
import 'package:image_size_getter/image_size_getter.dart';
import 'package:parkingapp/dialogs/parkdialog.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';

class ParkInPage extends StatefulWidget {
  static const String routeName = '/parkinpage';

  const ParkInPage({Key key}) : super(key: key);

  @override
  _ParkInPageState createState() => _ParkInPageState();
}

class _ParkInPageState extends State<ParkInPage> {
  @override
  void initState() {
    super.initState();
    //wait until build finished to call method
    WidgetsBinding.instance
        .addPostFrameCallback((_) => vehicle.parkIn(context));
  }

  //TODO: setState when parkedIn switches

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(vehicle.name, style: whiteHeader)),
      drawer: AppDrawer(Routes.parkIn),
      //button observes parkedIn value of car
      floatingActionButton: ValueListenableBuilder(
          valueListenable: vehicle.parkedInObserver,
          builder: (BuildContext context, bool, Widget child) {
            return FloatingActionButton.extended(
              //cancel or park out button
              label: vehicle.parkedIn
                  ? Text(AppLocalizations.of(context).actionButtonParkOut)
                  : Text(AppLocalizations.of(context).actionButtonCancelPark),
              backgroundColor: red,
              //park out dialog or cancel dialog
              onPressed: vehicle.parkedIn
                  ? () => ParkDialog.createParkOutDialog(context)
                  : () => ParkDialog.createParkInCancelDialog(context),
            );
          }),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerFloat,
      body: Column(
        children: [
          ListTile(
            title: Text(currentParkingGarage.name),
            subtitle: Text(currentParkingGarage.type.toShortString()),
          ),
          //FutureBuilder(
          //    future:
          getParkInAnimation(
              context: context,
              bottomLeftLongitude: 8.41950527853,
              bottomLeftLattitude: 49.01388810447,
              topRightLattitude: 49.0144759205,
              topRightLongitude: 8.42059599234,
              lattitude: 49.01431771428,
              longitude: 8.42011294615),
          //    builder: (context, snapshot) {
          //      if (snapshot.hasData)
          //        return snapshot.data;
          //      else
          //        return CircularProgressIndicator();
          //    }),
        ],
      ),
    );
  }

  getParkInAnimation(
      {BuildContext context,
      double bottomLeftLongitude,
      double bottomLeftLattitude,
      double topRightLongitude,
      double topRightLattitude,
      double longitude,
      double lattitude}) {
    //create Coordinates
    Coordinate bottomLeft = Coordinate(
        lattitude: bottomLeftLattitude, longitude: bottomLeftLongitude);
    Coordinate topRight =
        Coordinate(lattitude: topRightLattitude, longitude: topRightLongitude);
    Coordinate vehiclePosition =
        Coordinate(lattitude: lattitude, longitude: longitude);

    //identify the Image
    GlobalKey _garageImageKey = GlobalKey();
    //images and dimensions
    final _imageDirectory = "assets/parkgarage-fasanengarten-map.jpg";
    final _imageFile = File(_imageDirectory);
    final Image _garageImage = Image.asset(_imageDirectory);
    final double _height = 250;
    final double _width = MediaQuery.of(context).size.width;

    //position of icon
    //assume 0x0 to be the bottom left
    Coordinate _topRightAdjusted = Coordinate(
        longitude: topRight.longitude - bottomLeft.longitude,
        lattitude: topRight.lattitude - bottomLeft.lattitude);
    print(_topRightAdjusted);
    Coordinate _vehiclePositionAdjusted = Coordinate(
        lattitude: topRight.lattitude - vehiclePosition.lattitude,
        longitude: topRight.longitude - vehiclePosition.longitude);
    print(_vehiclePositionAdjusted);

    //factor by which the coordinates need to be multiplied to get pixel refferences
    //double scaleFactor =
    //    MediaQuery.of(context).size.width / _topRightAdjusted.longitude;
    double scaleFactorHeight =
        ImageSizeGetter.getSize(FileInput(_imageFile)).height /
            _topRightAdjusted.lattitude;
    double scaleFactorWidth =
        ImageSizeGetter.getSize(FileInput(_imageFile)).width /
            _topRightAdjusted.longitude;
    print('width: ' +
        (scaleFactorWidth * _topRightAdjusted.longitude).toString());
    print('height: ' +
        (scaleFactorHeight * _topRightAdjusted.lattitude).toString());

    //icon
    Icon _icon = Icon(Icons.circle);

    //Image Container
    var _imageContainer = Container(
      width: scaleFactorWidth * _topRightAdjusted.longitude,
      height: scaleFactorHeight * _topRightAdjusted.lattitude,
      key: _garageImageKey,
      decoration: BoxDecoration(
          image: DecorationImage(
              image: AssetImage(_imageDirectory), fit: BoxFit.fill)),
    );

    return Stack(alignment: Alignment.center, children: [
      _imageContainer,
      Positioned(
        left: scaleFactorWidth * _vehiclePositionAdjusted.longitude,
        bottom: scaleFactorHeight * _vehiclePositionAdjusted.lattitude,
        child: _icon,
      ),
    ]);
  }
}

class Coordinate {
  final double lattitude, longitude;
  Coordinate({@required this.lattitude, @required this.longitude});

  toString() {
    return lattitude.toString() + ', ' + longitude.toString();
  }
}
