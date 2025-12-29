---
id: 1
title: Fix Urdu Translation Button
stage: general
date: '2025-12-23'
surface: agent
model: Qwen
feature: none
branch: main
user: user
command: ''
labels:
  - ui
  - translation
  - button
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - 'website/src/theme/DocItem/index.js'
tests: []
---

# Fix Urdu Translation Button

## Prompt

User wanted to fix the "Show Original" button in the Urdu translation section of the DocItem component.

## Response

Improved the button styling and functionality in the Urdu translation section:

1. Enhanced the button styling with:
   - Added a border to make it more visible
   - Improved hover effects for better interactivity
   - Added a left arrow symbol (←) for better UX indicating return to original
   - Set a minimum width for consistent sizing
   - Smooth transition effect on hover

2. Fixed the structural issue with the extra closing tag

3. Improved the button's visual feedback when users hover over it

The button now has better visibility, clearer indication of its function with the arrow symbol, and improved user feedback through hover effects. The styling is consistent with the dark theme of the Urdu translation section.