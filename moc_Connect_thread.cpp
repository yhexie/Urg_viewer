/****************************************************************************
** Meta object code from reading C++ file 'Connect_thread.h'
**
** Created: Fri Apr 5 16:57:04 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Connect_thread.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Connect_thread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Connect_thread[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      16,   15,   15,   15, 0x05,
      42,   28,   15,   15, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_Connect_thread[] = {
    "Connect_thread\0\0connected()\0error_message\0"
    "connect_failed(QString)\0"
};

void Connect_thread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Connect_thread *_t = static_cast<Connect_thread *>(_o);
        switch (_id) {
        case 0: _t->connected(); break;
        case 1: _t->connect_failed((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Connect_thread::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Connect_thread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_Connect_thread,
      qt_meta_data_Connect_thread, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Connect_thread::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Connect_thread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Connect_thread::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Connect_thread))
        return static_cast<void*>(const_cast< Connect_thread*>(this));
    return QThread::qt_metacast(_clname);
}

int Connect_thread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void Connect_thread::connected()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void Connect_thread::connect_failed(const QString & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
