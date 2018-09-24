create table events
(
    ID serial primary key,
    timestamp TIMESTAMP NOT NULL,
    subject varchar(40) NOT NULL,
    verb    varchar(20) NOT NULL,
    directobject varchar(40),
    data    varchar(256)
);
