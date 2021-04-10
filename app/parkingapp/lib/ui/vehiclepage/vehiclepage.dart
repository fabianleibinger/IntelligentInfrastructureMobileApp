import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/bloc/events/deletevehicle.dart';
import 'package:parkingapp/dialogs/deletevehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/models/widgets/expandableFloatingActionButton.dart';
import 'package:parkingapp/ui/editvehicle/editvehicle.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/util/vehiclehelper.dart';

class VehiclePage extends StatefulWidget {
  static const String routeName = '/vehiclepage';

  const VehiclePage({Key key}) : super(key: key);

  @override
  _VehiclePageState createState() => _VehiclePageState();
}

class _VehiclePageState extends State<VehiclePage> {
  final GlobalKey<FormState> _loginFormKey =
      new GlobalKey<FormState>(debugLabel: '_loginFormKey');
  static bool _firstBuild = true;

  @override
  void initState() {
    super.initState();
    //only init vehicles after app start up
    if (_firstBuild) {
      DataHelper.initVehicles(context);
      _firstBuild = false;
    }
    VehicleHelper.cleanUpVehicles(context);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      key: _loginFormKey,
        appBar: AppBar(
          title: Text(AppLocalizations.of(context).drawerVehicles),
        ),
        //TODO use routeName
        drawer: AppDrawer(),
        floatingActionButton: FancyFab(),
        body: Container(padding: EdgeInsets.all(8), child: createListView()));
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
              return Dismissible(
                  confirmDismiss: (dismissDirection) =>
                      //confirm to delete
                      //will not allow deletion if vehicle is parked in
                      showDialog(
                          context: context,
                          builder: (_) => vehicle.parkedIn ||
                                  vehicle.parkingIn ||
                                  vehicle.parkingOut
                              ? CantDeleteVehicle()
                              : ConfirmDelete()),
                  direction: DismissDirection.endToStart,
                  background: Container(
                    padding: EdgeInsets.all(10),
                    child: Align(
                      alignment: Alignment.centerRight,
                      child: Icon(Icons.delete,
                          color: Theme.of(context).dialogBackgroundColor),
                    ),
                    color: Theme.of(context).errorColor,
                  ),
                  key: Key(vehicle.inAppKey),
                  onDismissed: (direction) {
                    //remove vehicle from displayed list
                    vehicleList.remove(vehicle.inAppKey);
                    //remove vehicle from database and bloc
                    DatabaseProvider.db.delete(vehicle.databaseId);
                    BlocProvider.of<VehicleBloc>(context)
                        .add(DeleteVehicle(vehicle));
                    //show SnackBar that vehicle has been deleted
                    ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                      content: Text(vehicle.name +
                          AppLocalizations.of(context).space +
                          AppLocalizations.of(context).deleted),
                    ));
                  },
                  child: ListTile(
                    title: Text(vehicle.name),
                    subtitle: Text(vehicle.licensePlate),
                    onTap: () => Navigator.of(context).push(MaterialPageRoute(
                        builder: (context) => EditVehicle(
                              vehicle: vehicle,
                            ))),
                  ));
            },
            itemCount: vehicleList.length);
      },
    );
  }
}
