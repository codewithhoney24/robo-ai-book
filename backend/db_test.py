import asyncio
import asyncpg

# Test database connection directly with asyncpg
async def db_test():
    # Database URL from the .env in backend
    database_url = "postgresql://neondb_owner:npg_gPuCILYeV73E@ep-square-glitter-a854b5sd-pooler.eastus2.azure.neon.tech/neondb"

    try:
        conn = await asyncpg.connect(database_url)

        # Run the sanity test query (without created_at since it's not in the table by default)
        rows = await conn.fetch("""
            SELECT email, software_background, hardware_background
            FROM "user"
            LIMIT 5;
        """)

        if rows:
            print('DB SANITY TEST: Found', len(rows), 'users in the database')
            print('Sample users:')
            for i, row in enumerate(rows, 1):
                print(f'  {i}. Email: {row["email"]}, Software: {row["software_background"]}, Hardware: {row["hardware_background"]}')
        else:
            print('DB SANITY TEST: No users found in the database')
            print('This means either no signups have occurred or there might be an issue with the signup API')

        await conn.close()

    except Exception as e:
        print(f'DB SANITY TEST: Error connecting or querying: {e}')

if __name__ == '__main__':
    asyncio.run(db_test())