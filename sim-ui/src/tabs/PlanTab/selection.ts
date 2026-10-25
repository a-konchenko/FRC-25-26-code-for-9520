// What is selected / being dragged in the path editor.
export type Selection =
  | { type: 'anchor' | 'prev' | 'next'; i: number }
  | { type: 'rot'; i: number }
  | { type: 'rotHandle'; i: number }
  | { type: 'marker'; i: number }
  | { type: 'start' | 'end' }
  | null;
