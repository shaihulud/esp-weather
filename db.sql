CREATE TABLE room_mine (
    dt              timestamp (0) with time zone DEFAULT NOW(),
    temperature_bme real,
    pressure_bme    real,
    humidity_bme    real,
    temperature_sht real,
    humidity_sht    real,
    co2             smallint
);
CREATE UNIQUE INDEX dt_idx ON room_mine (dt);

CREATE ROLE esp32_1 WITH LOGIN, PASSWORD '';
GRANT INSERT ON room_mine TO esp32_1;

CREATE ROLE grafana WITH LOGIN, PASSWORD '';
GRANT SELECT ON room_mine TO grafana;


CREATE TABLE exchange_rate (
    dt        timestamp (0) with time zone DEFAULT NOW(),
    currency  character varying(3),
    "value"   real
);
CREATE UNIQUE INDEX idx_exchange_rate_dt_currency ON exchange_rate (dt, currency);

ALTER ROLE currency_bot WITH LOGIN;
GRANT INSERT ON exchange_rate TO currency_bot;