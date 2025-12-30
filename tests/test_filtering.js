// Test the filtering logic directly
console.log('Testing personalization filtering logic...');

// Mock data similar to what's in PersonalizedDocsList
const mockDocs = [
  { id: 'doc1', title: 'Beginner Doc', difficulty: 'beginner' },
  { id: 'doc2', title: 'Intermediate Doc', difficulty: 'intermediate' },
  { id: 'doc3', title: 'Advanced Doc', difficulty: 'advanced' },
];

// Test 1: Beginner level should only show beginner content
const beginnerFiltered = mockDocs.filter(doc => {
  if ('beginner' === 'beginner' && doc.difficulty !== 'beginner') {
    return false;
  } else if ('beginner' === 'intermediate' && doc.difficulty === 'advanced') {
    return false;
  }
  return true;
});

console.log('Beginner level filtering result:', beginnerFiltered);
console.log('Expected: Only beginner doc, Actual:', beginnerFiltered.length === 1 && beginnerFiltered[0].difficulty === 'beginner' ? 'PASS' : 'FAIL');

// Test 2: Intermediate level should show beginner and intermediate content
const intermediateFiltered = mockDocs.filter(doc => {
  if ('intermediate' === 'beginner' && doc.difficulty !== 'beginner') {
    return false;
  } else if ('intermediate' === 'intermediate' && doc.difficulty === 'advanced') {
    return false;
  }
  return true;
});

console.log('Intermediate level filtering result:', intermediateFiltered);
console.log('Expected: Beginner and intermediate docs, Actual:', intermediateFiltered.length === 2 && 
  intermediateFiltered.some(d => d.difficulty === 'beginner') && 
  intermediateFiltered.some(d => d.difficulty === 'intermediate') ? 'PASS' : 'FAIL');

// Test 3: Advanced level should show all content
const advancedFiltered = mockDocs.filter(doc => {
  if ('advanced' === 'beginner' && doc.difficulty !== 'beginner') {
    return false;
  } else if ('advanced' === 'intermediate' && doc.difficulty === 'advanced') {
    return false;
  }
  return true;
});

console.log('Advanced level filtering result:', advancedFiltered);
console.log('Expected: All docs, Actual:', advancedFiltered.length === 3 ? 'PASS' : 'FAIL');

console.log('Testing completed.');