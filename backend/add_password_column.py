import asyncio
from sqlalchemy import create_engine, text
from src.config.settings import settings
from src.utils.db_url_parser import clean_database_url_for_asyncpg

def add_password_column():
    # Clean the database URL
    database_url = settings.neon_database_url
    clean_database_url = clean_database_url_for_asyncpg(database_url)
    
    print(f"Using database URL: {clean_database_url.replace('@', '[@]').split('@')[-1] if '@' in clean_database_url else clean_database_url}")
    
    # Create sync engine
    engine = create_engine(clean_database_url)
    
    try:
        # Connect and add the password column if it doesn't exist
        with engine.connect() as conn:
            # Check if password column exists
            result = conn.execute(text("""
                SELECT column_name 
                FROM information_schema.columns 
                WHERE table_name = 'user' AND column_name = 'password'
            """))
            password_col_exists = result.fetchone() is not None
            
            if not password_col_exists:
                # Add the password column
                conn.execute(text("ALTER TABLE \"user\" ADD COLUMN password TEXT;"))
                conn.commit()
                print("Added password column to user table")
            else:
                print("Password column already exists")
                
            # Check all columns again
            result = conn.execute(text("SELECT column_name FROM information_schema.columns WHERE table_name = 'user' ORDER BY ordinal_position;"))
            columns = [row[0] for row in result.fetchall()]
            print(f'Updated columns in user table: {columns}')
                
    except Exception as e:
        print(f"Error modifying database: {e}")
        if 'conn' in locals():
            conn.rollback()

if __name__ == "__main__":
    add_password_column()