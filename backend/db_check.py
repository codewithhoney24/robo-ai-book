import asyncio
from sqlalchemy import create_engine, text
from sqlalchemy.dialects.postgresql import ARRAY
from src.config.settings import settings
from src.utils.db_url_parser import clean_database_url_for_asyncpg

def check_db():
    # Clean the database URL
    database_url = settings.neon_database_url
    clean_database_url = clean_database_url_for_asyncpg(database_url)
    
    print(f"Using database URL: {clean_database_url.replace('@', '[@]').split('@')[-1] if '@' in clean_database_url else clean_database_url}")
    
    # Create sync engine
    engine = create_engine(clean_database_url)
    
    try:
        # Test connection and check if table exists
        with engine.connect() as conn:
            # Check if user table exists
            result = conn.execute(text("SELECT EXISTS (SELECT FROM information_schema.tables WHERE table_name = 'user');"))
            table_exists = result.scalar()
            print(f'User table exists: {table_exists}')
            
            if table_exists:
                # Check columns
                result = conn.execute(text("SELECT column_name, data_type FROM information_schema.columns WHERE table_name = 'user' ORDER BY ordinal_position;"))
                columns = result.fetchall()
                print(f'Columns in user table:')
                for col_name, data_type in columns:
                    print(f'  - {col_name}: {data_type}')
            
            # Try to query the table structure directly
            try:
                result = conn.execute(text("SELECT * FROM \"user\" LIMIT 0;"))
                print(f"\nTable structure query successful")
                print(f"Available columns from reflection: {[col.name for col in result.cursor.description]}")
            except Exception as e:
                print(f"\nError querying table structure: {e}")
                
    except Exception as e:
        print(f"Database connection error: {e}")

if __name__ == "__main__":
    check_db()