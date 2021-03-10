from flask import Flask
from flask_sqlalchemy import SQLAlchemy

db = SQLAlchemy()

class IDMapping(db.Model):
    """
    This class defines the two rows, app_id and parkhaus_id, for the database,
    which maps the two of them together.
    """
    __tablename__ = 'id_mapping'

    app_id = db.Column(db.String(), primary_key=True)
    parkhaus_id = db.Column(db.String())

    def __init__(self, app_id, parkhaus_id):
        """
        This method initializes the params of a IDMapping object.
        :param app_id: The ID of the vehicle specified by the app.
        :param parkhaus_id: The ID of the vehicle specified by the parkhausmanagmentsystem.
        """
        self.app_id = app_id
        self.parkhaus_id = parkhaus_id
    
    
    def serialize(self):
        """
        This method returns the params of a IDMapping object in a JSON format.
        :return: The params of a IDMapping object in a JSON format.
        """
        return {
            'app_id' : self.app_id,
            'parkhaus_id' : self.parkhaus_id
        }
    
    def getParkhausID(self):
        """
        Getter for parkhaus_id.
        :return: The parkhaus_id.
        """
        return self.parkhaus_id


def init_db():
    """
    This method initializes the database.
    """
    db.create_all()

def clear_db():
    """
    This method clears the database by dropping all tables.
    """
    db.drop_all()

def add(app_id, parkhaus_id):
    """
    This method adds a new entry into the database. 
    If the app_id already exists in the database it will delete the existing one, 
    before adding the new one.
    :param app_id: The ID of the vehicle specified by the app.
    :param parkhaus_id: The ID of the vehicle specified by the parkhausmanagmentsystem.
    :return: The feedback wether the app_id already existed.
    """
    idMapping = IDMapping.query.filter_by(app_id=app_id).first()
    if not idMapping:
        db.session.add(IDMapping(app_id, parkhaus_id))
        db.session.commit()
        return False
    else:
        IDMapping.query.filter(IDMapping.app_id == app_id).delete()
        db.session.add(IDMapping(app_id, parkhaus_id))
        db.session.commit()
        return True

def getParkhausID(app_id):
    """
    This method returns the corresponding parkhaus_id for a given app_id.
    :param app_id: The ID of the vehicle specified by the app.
    :return: The corresponding parkhaus_id for the app_id.
    """
    return IDMapping.getParkhausID(IDMapping.query.filter_by(app_id=app_id).first())