/****************************************************************************
** Meta object code from reading C++ file 'pclobjectextractor.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "pclobjectextractor.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pclobjectextractor.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PCLObjectExtractor[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
      31,   20,   19,   19, 0x05,
      70,   57,   19,   19, 0x05,
     108,   20,   19,   19, 0x05,
     131,   57,   19,   19, 0x05,

 // slots: signature, parameters, type, tag, flags
     166,   20,   19,   19, 0x08,
     190,   57,   19,   19, 0x08,
     226,   20,   19,   19, 0x08,
     247,   57,   19,   19, 0x08,
     280,   19,   19,   19, 0x08,
     306,   19,   19,   19, 0x08,
     332,   19,   19,   19, 0x08,
     356,   19,   19,   19, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_PCLObjectExtractor[] = {
    "PCLObjectExtractor\0\0pointIndex\0"
    "PointHighlightSignal(int)\0pointIndices\0"
    "AreaHighlightSignal(std::vector<int>)\0"
    "PointRemoveSignal(int)\0"
    "AreaRemoveSignal(std::vector<int>)\0"
    "PointHighlightSlot(int)\0"
    "AreaHighlightSlot(std::vector<int>)\0"
    "PointRemoveSlot(int)\0"
    "AreaRemoveSlot(std::vector<int>)\0"
    "on_actionHelp_triggered()\0"
    "on_actionExit_triggered()\0"
    "on_loadButton_clicked()\0on_saveButton_clicked()\0"
};

void PCLObjectExtractor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PCLObjectExtractor *_t = static_cast<PCLObjectExtractor *>(_o);
        switch (_id) {
        case 0: _t->PointHighlightSignal((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->AreaHighlightSignal((*reinterpret_cast< std::vector<int>(*)>(_a[1]))); break;
        case 2: _t->PointRemoveSignal((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->AreaRemoveSignal((*reinterpret_cast< std::vector<int>(*)>(_a[1]))); break;
        case 4: _t->PointHighlightSlot((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->AreaHighlightSlot((*reinterpret_cast< std::vector<int>(*)>(_a[1]))); break;
        case 6: _t->PointRemoveSlot((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->AreaRemoveSlot((*reinterpret_cast< std::vector<int>(*)>(_a[1]))); break;
        case 8: _t->on_actionHelp_triggered(); break;
        case 9: _t->on_actionExit_triggered(); break;
        case 10: _t->on_loadButton_clicked(); break;
        case 11: _t->on_saveButton_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData PCLObjectExtractor::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PCLObjectExtractor::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_PCLObjectExtractor,
      qt_meta_data_PCLObjectExtractor, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PCLObjectExtractor::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PCLObjectExtractor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PCLObjectExtractor::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PCLObjectExtractor))
        return static_cast<void*>(const_cast< PCLObjectExtractor*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int PCLObjectExtractor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    }
    return _id;
}

// SIGNAL 0
void PCLObjectExtractor::PointHighlightSignal(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void PCLObjectExtractor::AreaHighlightSignal(std::vector<int> _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void PCLObjectExtractor::PointRemoveSignal(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void PCLObjectExtractor::AreaRemoveSignal(std::vector<int> _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE
