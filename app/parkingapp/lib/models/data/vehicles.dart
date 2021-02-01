import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:sqflite/sqflite.dart';
import 'dart:async';
import 'package:path/path.dart';

class DatabaseProvider {
  static const String TABLE_VEHICLE = "vehicle";
  static const String COLUMN_ID = "id";
  static const String COLUMN_KEY = "key";
  static const String COLUMN_NAME = "name";
  static const String COLUMN_LICENSEPLATE = "licenseplate";
  static const String COLUMN_WIDTH = "width";
  static const String COLUMN_HEIGHT = "height";
  static const String COLUMN_DEPTH = "depth";
  static const String COLUMN_TURNANGLE = "turnangle";
  static const String COLUMN_NEAREXIT = "nearexit";
  static const String COLUMN_PARKINGCARD = "parkingcard";
  static const String COLUMN_DOCHARGE = "docharge";
  static const String COLUMN_CHARGINGPROVIDER = "chargingprovider";
  static const String COLUMN_CHARGEBEGIN = "chargebegin";
  static const String COLUMN_CHARGEEND = "chargeend";
  static const String COLUMN_CHARGE = "charge";

  DatabaseProvider._();
  static final DatabaseProvider db = DatabaseProvider._();

  Database _database;

  Future<Database> get database async {
    print("database getter called");

    if (_database != null) {
      return _database;
    }

    _database = await createDatabase();
    return _database;
  }

  Future<Database> createDatabase() async {
    String dbPath = await getDatabasesPath();

    return await openDatabase(
      join(dbPath, 'vehicleDB.db'),
      version: 1,
      onCreate: (Database database, int version) async {
        print("Creating vehicle table");

        await database.execute(
          "CREATE TABLE $TABLE_VEHICLE ("
          "$COLUMN_ID INTEGER PRIMARY KEY,"
          "$COLUMN_KEY TEXT,"
          "$COLUMN_NAME TEXT,"
          "$COLUMN_LICENSEPLATE TEXT,"
          "$COLUMN_WIDTH DOUBLE,"
          "$COLUMN_HEIGHT DOUBLE,"
          "$COLUMN_DEPTH DOUBLE,"
          "$COLUMN_TURNANGLE DOUBLE,"
          "$COLUMN_NEAREXIT INTEGER,"
          "$COLUMN_PARKINGCARD INTEGER,"
          "$COLUMN_DOCHARGE INTEGER,"
          "$COLUMN_CHARGINGPROVIDER TEXT,"
          "$COLUMN_CHARGEBEGIN TEXT,"
          "$COLUMN_CHARGEEND TEXT,"
          "$COLUMN_NEAREXIT TEXT"
          ")",
        );
      },
    );
  }

  Future<List<ElectricalVehicle>> getVehicles() async {
    final db = await database;

    var vehicles = await db.query(TABLE_VEHICLE, columns: [
      COLUMN_ID,
      COLUMN_KEY,
      COLUMN_NAME,
      COLUMN_LICENSEPLATE,
      COLUMN_WIDTH,
      COLUMN_HEIGHT,
      COLUMN_DEPTH,
      COLUMN_TURNANGLE,
      COLUMN_NEAREXIT,
      COLUMN_PARKINGCARD,
      COLUMN_DOCHARGE,
      COLUMN_CHARGINGPROVIDER,
      COLUMN_CHARGEBEGIN,
      COLUMN_CHARGEEND,
      COLUMN_CHARGE
    ]);

    List<ElectricalVehicle> vehicleList = List<ElectricalVehicle>();

    vehicles.forEach((currentVehicle) {
      ElectricalVehicle vehicle = ElectricalVehicle.fromMap(currentVehicle);
      vehicleList.add(vehicle);
    });

    return vehicleList;
  }

  Future<ElectricalVehicle> insert(ElectricalVehicle vehicle) async {
    final db = await database;
    vehicle.id = await db.insert(TABLE_VEHICLE, vehicle.toMap());
    return vehicle;
  }

  Future<int> delete(int id) async {
    final db = await database;
    return await db.delete(TABLE_VEHICLE, where: "id = ?", whereArgs: [id]);
  }

  Future<int> update(ElectricalVehicle vehicle) async {
    final db = await database;
    return await db.update(TABLE_VEHICLE, vehicle.toMap(),
        where: "id = ?", whereArgs: [vehicle.id]);
  }
}
