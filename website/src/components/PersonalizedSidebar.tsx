import React, { useState, useEffect } from 'react';
import { usePersonalization } from '../contexts/PersonalizationContext';
import Link from '@docusaurus/Link';
import clsx from 'clsx';

interface DocItem {
  type: string;
  label?: string;
  href?: string;
  id?: string;
  items?: DocItem[];
  collapsed?: boolean;
}

const PersonalizedSidebar: React.FC<{ sidebarItems?: DocItem[] }> = ({ sidebarItems = [] }) => {
  const { personalizationData } = usePersonalization();
  const { preferences } = personalizationData;
  const [activeSidebar, setActiveSidebar] = useState<DocItem[]>([]);

  // Filter sidebar items based on personalization preferences
  useEffect(() => {
    if (sidebarItems && sidebarItems.length > 0) {
      const filteredItems = filterSidebarItems(sidebarItems, preferences);
      setActiveSidebar(filteredItems);
    }
  }, [sidebarItems, preferences, preferences.difficulty, preferences.preferredTopics, preferences.contentContext, preferences.hardwareBackground]);

  // Function to filter sidebar items based on personalization
  const filterSidebarItems = (items: DocItem[], prefs: any): DocItem[] => {
    return items
      .map(item => {
        // If it's a category with sub-items
        if (item.type === 'category' && item.items) {
          const filteredSubItems = filterSidebarItems(item.items, prefs);

          // Only include the category if it has sub-items after filtering
          if (filteredSubItems.length > 0) {
            return {
              ...item,
              items: filteredSubItems
            };
          }
        }
        // If it's a regular doc item
        else if (item.type === 'doc' || item.type === 'link') {
          // Check if this item should be included based on preferences
          if (shouldIncludeItem(item, prefs)) {
            return item;
          }
        }

        return null;
      })
      .filter(Boolean) as DocItem[];
  };

  // Function to determine if an item should be included based on preferences
  const shouldIncludeItem = (item: DocItem, prefs: any): boolean => {
    // Extract difficulty from the item ID if possible
    const itemDifficulty = extractDifficultyFromId(item.id || '');

    // Filter by difficulty level according to the requirements:
    // Beginner: only beginner content
    // Intermediate: beginner and intermediate content
    // Advanced: all content
    if (prefs.difficulty === 'beginner' && itemDifficulty && itemDifficulty !== 'beginner') {
      return false;
    } else if (prefs.difficulty === 'intermediate' && itemDifficulty === 'advanced') {
      return false;
    }

    // If preferred topics are specified, check if the item matches
    if (prefs.preferredTopics && prefs.preferredTopics.length > 0) {
      // Check if item label or ID contains any of the preferred topics
      const itemText = (item.label || item.id || '').toLowerCase();
      const matchesTopic = prefs.preferredTopics.some((topic: string) =>
        itemText.includes(topic.toLowerCase())
      );
      if (!matchesTopic) {
        return false;
      }
    }

    // Filter by content context
    if (prefs.contentContext && !item.label?.toLowerCase().includes(prefs.contentContext.toLowerCase())) {
      return false;
    }

    // Filter by hardware background
    if (prefs.hardwareBackground && prefs.hardwareBackground !== 'none') {
      // If hardware background is specified, check if the item is relevant
      const isHardwareRelated = item.label?.toLowerCase().includes('hardware') ||
                                item.label?.toLowerCase().includes('sensor') ||
                                item.label?.toLowerCase().includes('kit') ||
                                item.id?.toLowerCase().includes('hardware') ||
                                item.id?.toLowerCase().includes('sensor') ||
                                item.id?.toLowerCase().includes('kit');

      if (prefs.hardwareBackground === 'beginner' && isHardwareRelated) {
        // For beginners, prefer docs without specific hardware requirements
        return false;
      } else if (prefs.hardwareBackground === 'advanced' && !isHardwareRelated) {
        // For advanced users, prefer docs with hardware focus
        return false;
      }
    }

    return true;
  };

  // Helper function to extract difficulty from item ID
  const extractDifficultyFromId = (id: string): string | null => {
    if (!id) return null;

    if (id.includes('m0-') || id.includes('m1-') || id.includes('foundations')) {
      return 'beginner';
    } else if (id.includes('m2-') || id.includes('m3-') || id.includes('edge-kit')) {
      return 'intermediate';
    } else if (id.includes('m4-') || id.includes('humanoid-mechanics') || id.includes('rtx-workstations')) {
      return 'advanced';
    }

    return null;
  };

  return (
    <div className="menu">
      <nav className="menu__list">
        {activeSidebar.map((item, index) => (
          <SidebarItem key={index} item={item} />
        ))}
      </nav>
    </div>
  );
};

// Recursive component to render sidebar items
const SidebarItem: React.FC<{ item: DocItem }> = ({ item }) => {
  if (item.type === 'category') {
    return (
      <div className="menu__list-item">
        <div className="menu__list-item-collapsible">
          <h4 className="menu__list-item-collapsible-title">
            {item.label}
          </h4>
        </div>
        <ul className="menu__list">
          {item.items?.map((subItem, index) => (
            <li key={index} className="menu__list-item">
              <SidebarItem item={subItem} />
            </li>
          ))}
        </ul>
      </div>
    );
  } else if (item.type === 'link') {
    return (
      <Link
        className={clsx('menu__link', 'menu__link--sublist')}
        to={item.href}
      >
        {item.label}
      </Link>
    );
  } else if (item.type === 'doc') {
    return (
      <Link
        className="menu__link"
        to={`/module/${item.id}`}
      >
        {item.label || item.id}
      </Link>
    );
  }

  return null;
};

export default PersonalizedSidebar;