import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/bloc/events/addvehicle.dart';
import 'package:parkingapp/bloc/events/setvehicles.dart';
import 'package:parkingapp/bloc/events/vehicleevent.dart';
import 'package:parkingapp/dialogs/scanqrdialog.dart';
import 'package:parkingapp/dialogs/drivesourcedialog.dart';
import 'package:parkingapp/dialogs/parkpreferencesdialog.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'package:parkingapp/models/widgets/expandableFloatingActionButton.dart';
import 'package:parkingapp/util/utility.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
// example for a page (mainpage)

class VehiclePage extends StatefulWidget {
  static const String routeName = '/vehiclepage';

  const VehiclePage({Key key}) : super(key: key);

  @override
  _VehiclePageState createState() => _VehiclePageState();
}

class _VehiclePageState extends State<VehiclePage> {
  final GlobalKey<FormState> _loginFormKey =
      new GlobalKey<FormState>(debugLabel: '_loginFormKey');
  @override
  void initState() {
    // TODO: implement initState
    super.initState();
    DatabaseProvider.db.getVehicles().then((vehicleList) {
      BlocProvider.of<VehicleBloc>(context).add(SetVehicles(vehicleList));
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
          title: Text('Vehicle', style: whiteHeader),
        ),
        //TODO use routeName
        drawer: AppDrawer(),
        floatingActionButton: FancyFab(),
        body: Container(
            padding: EdgeInsets.all(8), color: white, child: createListView()));
  }

  Widget createListView() {
    return BlocBuilder<VehicleBloc, List<Vehicle>>(
      buildWhen: (List<Vehicle> previous, List<Vehicle> current) {
        return true;
      },
      builder: (context, vehicleList) {
        return ListView.separated(
            separatorBuilder: (context, index) => Divider(
                  color: lightgrey,
                  thickness: 1.5,
                  height: 5,
                  indent: 15,
                  endIndent: 15,
                ),
            itemBuilder: (BuildContext context, int index) {
              print("vehicleList: $vehicleList");

              Vehicle vehicle = vehicleList[index];
              return ListTile(
                title: Text(vehicle.name),
                subtitle: Text(vehicle.licensePlate +
                    "; " +
                    vehicle.databaseId.toString()),
                //onTap: () => showVehicleDialog(context, vehicle, index),
              );
            },
            itemCount: vehicleList.length);
      },
    );
  }
}
