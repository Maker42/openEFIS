echo "Start the Server"
pg_ctl start -D /usr/local/pgsql/data
echo "Create the Table"
psql -f create_events_table.sql flight
echo "Start EventDB microserver"
python3 EventDB.py flight postgres events
