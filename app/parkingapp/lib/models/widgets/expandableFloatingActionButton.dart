import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/bloc/events/addvehicle.dart';
import 'package:parkingapp/bloc/events/deletevehicle.dart';
import 'package:parkingapp/bloc/events/resetvehicles.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/editvehicle/editvehicle.dart';
import 'package:parkingapp/ui/vehiclepage/vehiclepage.dart';
import 'package:parkingapp/util/utility.dart';
import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';

class FancyFab extends StatefulWidget {
  final Function() onPressed;
  final String tooltip;
  final IconData icon;

  FancyFab({this.onPressed, this.tooltip, this.icon});

  @override
  _FancyFabState createState() => _FancyFabState();
}

class _FancyFabState extends State<FancyFab>
    with SingleTickerProviderStateMixin {
  bool isOpened = false;
  AnimationController _animationController;
  Animation<Color> _buttonColor;
  Animation<double> _animateIcon;
  Animation<double> _translateButton;
  Curve _curve = Curves.easeOut;
  double _fabHeight = 56.0;
  double spaceBetween = -6.0;

  @override
  initState() {
    _animationController =
        AnimationController(vsync: this, duration: Duration(milliseconds: 500))
          ..addListener(() {
            setState(() {});
          });
    _animateIcon =
        Tween<double>(begin: 0.0, end: 1.0).animate(_animationController);
    _buttonColor = ColorTween(
      begin: green,
      end: green,
    ).animate(CurvedAnimation(
      parent: _animationController,
      curve: Interval(
        0.00,
        1.00,
        curve: Curves.linear,
      ),
    ));
    _translateButton = Tween<double>(
      begin: _fabHeight,
      end: spaceBetween,
    ).animate(CurvedAnimation(
      parent: _animationController,
      curve: Interval(
        0.0,
        0.75,
        curve: _curve,
      ),
    ));
    super.initState();
  }

  @override
  dispose() {
    _animationController.dispose();
    super.dispose();
  }

  animate() {
    if (!isOpened) {
      _animationController.forward();
    } else {
      _animationController.reverse();
    }
    isOpened = !isOpened;
  }

  Widget inbox() {
    return Container(
      child: FloatingActionButton(
        onPressed: () {
          // DatabaseProvider.db.delete(2).then((index) {
          //   BlocProvider.of<VehicleBloc>(context).add(DeleteVehicle(index));
          // });
          DatabaseProvider.db
              .insert(ChargeableVehicle(
                  Utility.generateKey(),
                  "Tesla Model 3",
                  "KA-ST 930 E",
                  1223.93,
                  2934.23,
                  4529.3,
                  0,
                  true,
                  false,
                  false,
                  true,
                  "EnBW",
                  TimeOfDay.now(),
                  TimeOfDay.now()))
              .then((vehicle) {
            BlocProvider.of<VehicleBloc>(context).add(AddVehicle(vehicle));
          });
          // DatabaseProvider.db.clear().then((value) {
          //   BlocProvider.of<VehicleBloc>(context).add(ResetVehicles());
          // });
          // Navigator.push(
          //     (context),
          //     MaterialPageRoute(
          //         builder: (BuildContext context) => VehiclePage()));
        },
        tooltip: 'QR Code',
        elevation: 10,
        child: Icon(Icons.qr_code),
        mini: true,
        heroTag: 2,
      ),
    );
  }

  Widget add() {
    return Container(
      child: FloatingActionButton(
        onPressed: () => Navigator.pushNamed(context, Routes.createVehicle),
        tooltip: 'Add',
        elevation: 10,
        child: Icon(Icons.add),
        mini: true,
        heroTag: 1,
      ),
    );
  }

  Widget toggle() {
    return Container(
      child: FloatingActionButton(
        backgroundColor: _buttonColor.value,
        onPressed: animate,
        tooltip: 'Toggle',
        elevation: 10,
        child: AnimatedIcon(
          icon: AnimatedIcons.menu_close,
          progress: _animateIcon,
        ),
        heroTag: 0,
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisAlignment: MainAxisAlignment.end,
      children: <Widget>[
        Transform(
          transform: Matrix4.translationValues(
            0.0,
            _translateButton.value * 1.8,
            0.0,
          ),
          child: inbox(),
        ),
        Transform(
          transform: Matrix4.translationValues(
            0.0,
            _translateButton.value,
            0.0,
          ),
          child: add(),
        ),
        toggle(),
      ],
    );
  }
}
